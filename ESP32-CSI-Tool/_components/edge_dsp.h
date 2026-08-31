#ifndef ESP32_CSI_EDGE_DSP_H
#define ESP32_CSI_EDGE_DSP_H

/*
 * On-device DSP primitives for the breathing/motion detector.
 *
 * Design constraints this file answers to:
 *   - No dynamic allocation. Every buffer is a fixed-size struct member so the
 *     memory cost is known at compile time.
 *   - No FFT. We only ever ask "is there periodicity in 0.2-0.5 Hz?", which is
 *     7 specific frequencies, not a whole spectrum. A Goertzel bank answers
 *     exactly that question at ~2-3x fewer multiplies than a 256-point real FFT
 *     and, more importantly, needs no complex scratch buffer and places its bins
 *     at exact frequencies rather than at fs/N multiples.
 *   - Block-based, not streaming, so the result is bit-comparable against the
 *     Python reference implementation on the same recorded data.
 */

#include <math.h>
#include <string.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- pipeline geometry -------------------------------------------------- */

#define EDGE_FS_HZ            10.0f   /* uniform rate after time-binning       */
#define EDGE_BIN_US           100000  /* 100 ms per bin == 10 Hz               */
#define EDGE_WINDOW_N         256     /* 25.6 s analysis window                */
#define EDGE_DECISION_MS      10000   /* evaluate every 10 s                   */

#define EDGE_BREATH_LO_HZ     0.20f
#define EDGE_BREATH_HI_HZ     0.50f
#define EDGE_GOERTZEL_BINS    7       /* 0.20 .. 0.50 Hz in 0.05 Hz steps      */

/* ---- ring buffer -------------------------------------------------------- */

typedef struct {
    float v[EDGE_WINDOW_N];
    int   head;      /* next write position */
    int   filled;    /* saturates at EDGE_WINDOW_N */
} edge_ring_t;

static inline void edge_ring_init(edge_ring_t *r) {
    memset(r, 0, sizeof(*r));
}

static inline void edge_ring_push(edge_ring_t *r, float x) {
    r->v[r->head] = x;
    r->head = (r->head + 1) % EDGE_WINDOW_N;
    if (r->filled < EDGE_WINDOW_N) r->filled++;
}

static inline bool edge_ring_full(const edge_ring_t *r) {
    return r->filled >= EDGE_WINDOW_N;
}

/* Copies oldest-to-newest into out[]. Callers need chronological order because
 * both the IIR filter and Goertzel are order-dependent. */
static inline void edge_ring_linearize(const edge_ring_t *r, float *out) {
    int start = (r->head - r->filled + EDGE_WINDOW_N) % EDGE_WINDOW_N;
    for (int i = 0; i < r->filled; i++) {
        out[i] = r->v[(start + i) % EDGE_WINDOW_N];
    }
}

/* ---- running mean / variance (Welford) ---------------------------------- */

typedef struct {
    uint32_t n;
    float    mean;
    float    m2;
} edge_stat_t;

static inline void edge_stat_init(edge_stat_t *s) { memset(s, 0, sizeof(*s)); }

static inline void edge_stat_push(edge_stat_t *s, float x) {
    s->n++;
    float d = x - s->mean;
    s->mean += d / (float) s->n;
    s->m2   += d * (x - s->mean);
}

static inline float edge_stat_var(const edge_stat_t *s) {
    return (s->n > 1) ? s->m2 / (float) (s->n - 1) : 0.0f;
}

static inline float edge_stat_std(const edge_stat_t *s) {
    return sqrtf(edge_stat_var(s));
}

/* ---- biquad band-pass --------------------------------------------------- */

typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} edge_biquad_t;

/*
 * RBJ cookbook band-pass (constant 0 dB peak gain).
 *
 * Numerical note: at fs=10 Hz the breathing band sits at a normalized frequency
 * of only ~0.03, where direct-form-I biquads start losing precision in float32.
 * Coefficients are therefore derived in double and the state is kept in float,
 * which measured clean on the reference recording (see validate_embedded.py).
 * Decimating to ~2 Hz before filtering would relax this further if the band
 * ever moves lower.
 */
static inline void edge_biquad_init_bandpass(edge_biquad_t *f,
                                             double f_lo, double f_hi, double fs) {
    double f0    = sqrt(f_lo * f_hi);          /* geometric centre */
    double bw    = f_hi - f_lo;
    double q     = f0 / bw;
    double w0    = 2.0 * M_PI * f0 / fs;
    double alpha = sin(w0) / (2.0 * q);

    double b0 =  alpha;
    double b1 =  0.0;
    double b2 = -alpha;
    double a0 =  1.0 + alpha;
    double a1 = -2.0 * cos(w0);
    double a2 =  1.0 - alpha;

    f->b0 = (float)(b0 / a0);
    f->b1 = (float)(b1 / a0);
    f->b2 = (float)(b2 / a0);
    f->a1 = (float)(a1 / a0);
    f->a2 = (float)(a2 / a0);
    f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
}

static inline void edge_biquad_reset(edge_biquad_t *f) {
    f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
}

static inline float edge_biquad_step(edge_biquad_t *f, float x) {
    float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2
                        - f->a1 * f->y1 - f->a2 * f->y2;
    f->x2 = f->x1; f->x1 = x;
    f->y2 = f->y1; f->y1 = y;
    return y;
}

/* ---- Goertzel bank ------------------------------------------------------ */

typedef struct {
    float freq[EDGE_GOERTZEL_BINS];
    float coeff[EDGE_GOERTZEL_BINS];
} edge_goertzel_bank_t;

static inline void edge_goertzel_init(edge_goertzel_bank_t *g, float fs) {
    float step = (EDGE_BREATH_HI_HZ - EDGE_BREATH_LO_HZ) / (EDGE_GOERTZEL_BINS - 1);
    for (int i = 0; i < EDGE_GOERTZEL_BINS; i++) {
        g->freq[i]  = EDGE_BREATH_LO_HZ + step * i;
        g->coeff[i] = 2.0f * cosf(2.0f * (float) M_PI * g->freq[i] / fs);
    }
}

/* Single-bin Goertzel power over x[0..n-1]. Two state variables, one multiply
 * and two adds per sample -- no complex arithmetic, no scratch buffer. */
static inline float edge_goertzel_power(const float *x, int n, float coeff) {
    float s1 = 0.0f, s2 = 0.0f;
    for (int i = 0; i < n; i++) {
        float s = x[i] + coeff * s1 - s2;
        s2 = s1;
        s1 = s;
    }
    return s1 * s1 + s2 * s2 - coeff * s1 * s2;
}

typedef struct {
    float peak_freq;   /* Hz of the strongest breathing-band bin              */
    float peak_power;
    float concentration; /* peak / sum(all bins) -- 1/N = flat, 1 = single tone */
    float power[EDGE_GOERTZEL_BINS];
} edge_spectrum_t;

static inline void edge_goertzel_analyze(const edge_goertzel_bank_t *g,
                                         const float *x, int n,
                                         edge_spectrum_t *out) {
    float total = 0.0f;
    int   best  = 0;
    for (int i = 0; i < EDGE_GOERTZEL_BINS; i++) {
        float p = edge_goertzel_power(x, n, g->coeff[i]);
        if (p < 0.0f) p = 0.0f;          /* guard float round-off near zero */
        out->power[i] = p;
        total += p;
        if (p > out->power[best]) best = i;
    }
    out->peak_freq     = g->freq[best];
    out->peak_power    = out->power[best];
    out->concentration = (total > 1e-9f) ? out->power[best] / total
                                         : 0.0f;
}

/* ---- SNR against an out-of-band reference ------------------------------- */

/*
 * Breathing confidence needs a noise reference, and the Python version used the
 * median of the whole 0.05-1.0 Hz PSD. Computing a full PSD on-device just to
 * get that median would defeat the point of the Goertzel bank, so we use the
 * band's own spectral concentration instead: a real breathing tone puts its
 * energy in one or two adjacent bins, while broadband noise spreads flat across
 * all seven. Flat = 1/7 = 0.143; the decision threshold sits above that.
 */
static inline float edge_breathing_score(const edge_spectrum_t *s) {
    return s->concentration * (float) EDGE_GOERTZEL_BINS;  /* 1.0 == flat noise */
}

/* ---- autocorrelation cross-check ---------------------------------------- */

/*
 * Independent confirmation path. Breathing at 0.2-0.5 Hz means a period of
 * 2-5 s, i.e. a lag of 20-50 samples at 10 Hz. Cheap enough to run alongside
 * Goertzel and useful because the two methods fail differently.
 */
static inline float edge_autocorr_peak_freq(const float *x, int n, float fs,
                                            float *out_strength) {
    int lag_min = (int)(fs / EDGE_BREATH_HI_HZ);  /* 20 */
    int lag_max = (int)(fs / EDGE_BREATH_LO_HZ);  /* 50 */
    if (lag_max >= n) lag_max = n - 1;

    float r0 = 0.0f;
    for (int i = 0; i < n; i++) r0 += x[i] * x[i];
    if (r0 < 1e-9f) { if (out_strength) *out_strength = 0.0f; return 0.0f; }

    float best_r = -1e30f;
    int   best_lag = lag_min;
    for (int lag = lag_min; lag <= lag_max; lag++) {
        float r = 0.0f;
        for (int i = lag; i < n; i++) r += x[i] * x[i - lag];
        r /= (float)(n - lag);
        if (r > best_r) { best_r = r; best_lag = lag; }
    }
    if (out_strength) *out_strength = best_r / (r0 / (float) n);
    return fs / (float) best_lag;
}

#endif /* ESP32_CSI_EDGE_DSP_H */
