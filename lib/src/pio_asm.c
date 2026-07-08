/*
 * SPDX-License-Identifier: MIT
 * pio_asm — minimal PIO assembler for pio_sim. See pio_asm.h.
 */

#include "pio_asm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LABELS 32U
#define MAX_DEFINES 32U
#define MAX_LINE 256U
#define MAX_TOKENS 16U

typedef struct {
    char name[32];
    uint8_t index;
    bool is_public;
} label_t;

typedef struct {
    char name[32];
    uint32_t value;
} define_t;

typedef struct {
    pio_program_t *out;
    label_t labels[MAX_LABELS];
    uint8_t label_count;
    define_t defines[MAX_DEFINES];
    uint8_t define_count;
    int line_no;
} asm_ctx_t;

/* ── Small string helpers ──────────────────────────────────────────────────── */

/* ASCII-only lowercase: avoids ctype.h, whose glibc macro form trips
 * -Wdisabled-macro-expansion under -Weverything. PIO source is ASCII. */
static char lower_ascii(char c)
{
    if ((c >= 'A') && (c <= 'Z')) {
        return (char)(c + ('a' - 'A'));
    }
    return c;
}

static bool ieq(const char *a_in, const char *b_in)
{
    const char *a = a_in;
    const char *b = b_in;
    while (*a && *b) {
        if (lower_ascii(*a) != lower_ascii(*b)) {
            return false;
        }
        a++;
        b++;
    }
    return (*a == '\0') && (*b == '\0');
}

static bool parse_uint(const char *s_in, uint32_t *out)
{
    const char *s = s_in;
    if ((s == NULL) || (*s == '\0')) {
        return false;
    }
    uint32_t v = 0;
    int base = 10;
    if ((s[0] == '0') && ((s[1] == 'x') || (s[1] == 'X'))) {
        base = 16;
        s = &s[2];
        if (*s == '\0') {
            return false;
        }
    } else if ((s[0] == '0') && ((s[1] == 'b') || (s[1] == 'B'))) {
        base = 2;
        s = &s[2];
        if (*s == '\0') {
            return false;
        }
    } else {
        /* decimal */
    }
    for (; *s; s++) {
        uint32_t digit;
        if ((base == 2) && ((*s == '0') || (*s == '1'))) {
            digit = (uint32_t)(*s - '0');
        } else if ((base != 2) && (*s >= '0') && (*s <= '9')) {
            digit = (uint32_t)(*s - '0');
        } else if ((base == 16) && (*s >= 'a') && (*s <= 'f')) {
            digit = (uint32_t)(*s - 'a') + 10U;
        } else if ((base == 16) && (*s >= 'A') && (*s <= 'F')) {
            digit = (uint32_t)(*s - 'A') + 10U;
        } else {
            return false;
        }
        v = v * (uint32_t)base + digit;
    }
    *out = v;
    return true;
}

/* Tokenize `line` in place on whitespace and commas. Returns token count. Sets
 * *truncated when the line holds more than `max` tokens (so callers can reject it
 * rather than silently dropping the overflow, e.g. a trailing side-set/delay). */
static uint8_t tokenize(char *line, char *tok[], uint8_t max, bool *truncated)
{
    uint8_t n = 0;
    char *p = line;
    *truncated = false;
    while (*p) {
        while ((*p == ' ') || (*p == '\t') || (*p == ',')) {
            p++;
        }
        if (*p == '\0') {
            break;
        }
        if (n >= max) {
            *truncated = true; /* another token exists but the array is full */
            break;
        }
        tok[n] = p;
        n++;
        while (*p && (*p != ' ') && (*p != '\t') && (*p != ',')) {
            p++;
        }
        if (*p != '\0') {
            *p = '\0';
            p++;
        }
    }
    return n;
}

static void pa_set_error(asm_ctx_t *ctx, const char *msg)
{
    if (ctx->out->ok) {
        ctx->out->ok = false;
        ctx->out->error_line = ctx->line_no;
        (void)snprintf(ctx->out->error, PIO_ASM_ERR_LEN, "%s", msg);
    }
}

static int find_label(const asm_ctx_t *ctx, const char *name)
{
    for (uint8_t i = 0; i < ctx->label_count; i++) {
        if (strcmp(ctx->labels[i].name, name) == 0) {
            return (int)ctx->labels[i].index;
        }
    }
    return -1;
}

static bool find_define(const asm_ctx_t *ctx, const char *name, uint32_t *out)
{
    for (uint8_t i = 0; i < ctx->define_count; i++) {
        if (strcmp(ctx->defines[i].name, name) == 0) {
            *out = ctx->defines[i].value;
            return true;
        }
    }
    return false;
}

/* ── Expression evaluator (pioasm-compatible) ──────────────────────────────────
 * Integer expressions over `.define` symbols, labels, and literals, matching
 * real pioasm's operators and precedence. Tightest binding first:
 *   unary -  >  & | ^  >  * /  >  + -  >  << >>  >  ::(32-bit reverse)
 * (pioasm's grammar binds `::` loosest of all, so `::A + 1` is `::(A + 1)`.)
 * Operands: decimal / 0x / 0b literals, identifiers (define first, then label),
 * and parenthesised sub-expressions. Evaluated in int64, truncated to 32 bits at
 * the use site. Whitespace between tokens is ignored, so a spaced or glued form
 * (`label + 1` / `label+1`) both work. */
typedef struct {
    const asm_ctx_t *ctx;
    const char *p;
    bool ok;
} expr_t;

static void expr_skip_ws(expr_t *e)
{
    while ((*e->p == ' ') || (*e->p == '\t')) {
        e->p++;
    }
}

/* Host-side 32-bit reversal for the `::` operator (mirrors pio_sim.c reverse32). */
static uint32_t expr_reverse32(uint32_t v_in)
{
    uint32_t v = v_in;
    v = ((v & 0x55555555U) << 1U) | ((v >> 1U) & 0x55555555U);
    v = ((v & 0x33333333U) << 2U) | ((v >> 2U) & 0x33333333U);
    v = ((v & 0x0F0F0F0FU) << 4U) | ((v >> 4U) & 0x0F0F0F0FU);
    v = ((v & 0x00FF00FFU) << 8U) | ((v >> 8U) & 0x00FF00FFU);
    v = (v << 16U) | (v >> 16U);
    return v;
}

static int64_t expr_reverse(expr_t *e); /* lowest-precedence entry, fwd decl */

static int64_t expr_primary(expr_t *e)
{
    expr_skip_ws(e);
    if (*e->p == '(') {
        e->p++;
        int64_t v = expr_reverse(e);
        expr_skip_ws(e);
        if (*e->p == ')') {
            e->p++;
        } else {
            e->ok = false;
        }
        return v;
    }
    if ((*e->p >= '0') && (*e->p <= '9')) {
        char num[34];
        size_t k = 0;
        while (((*e->p >= '0') && (*e->p <= '9')) || ((*e->p >= 'a') && (*e->p <= 'f')) ||
               ((*e->p >= 'A') && (*e->p <= 'F')) || (*e->p == 'x') || (*e->p == 'X')) {
            if (k < (sizeof(num) - 1U)) {
                num[k] = *e->p;
                k++;
            }
            e->p++;
        }
        num[k] = '\0';
        uint32_t u = 0;
        if (!parse_uint(num, &u)) {
            e->ok = false;
            return 0;
        }
        return (int64_t)u;
    }
    if (((*e->p >= 'a') && (*e->p <= 'z')) || ((*e->p >= 'A') && (*e->p <= 'Z')) ||
        (*e->p == '_')) {
        char id[32];
        size_t k = 0;
        while (((*e->p >= 'a') && (*e->p <= 'z')) || ((*e->p >= 'A') && (*e->p <= 'Z')) ||
               ((*e->p >= '0') && (*e->p <= '9')) || (*e->p == '_')) {
            if (k < (sizeof(id) - 1U)) {
                id[k] = *e->p;
                k++;
            }
            e->p++;
        }
        id[k] = '\0';
        uint32_t dv = 0;
        if (find_define(e->ctx, id, &dv)) {
            return (int64_t)dv;
        }
        int lbl = find_label(e->ctx, id);
        if (lbl >= 0) {
            return (int64_t)lbl;
        }
        e->ok = false;
        return 0;
    }
    e->ok = false;
    return 0;
}

static int64_t expr_unary(expr_t *e)
{
    expr_skip_ws(e);
    if (*e->p == '-') {
        e->p++;
        return -expr_unary(e);
    }
    return expr_primary(e);
}

static int64_t expr_bit(expr_t *e) /* & | ^ — tightest binary */
{
    int64_t v = expr_unary(e);
    for (;;) {
        expr_skip_ws(e);
        char c = *e->p;
        if ((c == '&') || (c == '|') || (c == '^')) {
            e->p++;
            int64_t r = expr_unary(e);
            v = (c == '&') ? (v & r) : ((c == '|') ? (v | r) : (v ^ r));
        } else {
            break;
        }
    }
    return v;
}

static int64_t expr_mul(expr_t *e)
{
    int64_t v = expr_bit(e);
    for (;;) {
        expr_skip_ws(e);
        char c = *e->p;
        if (c == '*') {
            e->p++;
            v *= expr_bit(e);
        } else if (c == '/') {
            e->p++;
            int64_t r = expr_bit(e);
            if (r != 0) {
                v /= r;
            } else {
                e->ok = false;
            }
        } else {
            break;
        }
    }
    return v;
}

static int64_t expr_add(expr_t *e)
{
    int64_t v = expr_mul(e);
    for (;;) {
        expr_skip_ws(e);
        char c = *e->p;
        if (c == '+') {
            e->p++;
            v += expr_mul(e);
        } else if (c == '-') {
            e->p++;
            v -= expr_mul(e);
        } else {
            break;
        }
    }
    return v;
}

static int64_t expr_shift(expr_t *e) /* << >> — loosest binary */
{
    int64_t v = expr_add(e);
    for (;;) {
        expr_skip_ws(e);
        if ((e->p[0] == '<') && (e->p[1] == '<')) {
            e->p += 2;
            v = v << expr_add(e);
        } else if ((e->p[0] == '>') && (e->p[1] == '>')) {
            e->p += 2;
            v = v >> expr_add(e);
        } else {
            break;
        }
    }
    return v;
}

/* `::` (32-bit reverse) binds loosest of all, matching pioasm's grammar: it
 * applies to the whole expression to its right (right-recursive). */
static int64_t expr_reverse(expr_t *e)
{
    expr_skip_ws(e);
    if ((e->p[0] == ':') && (e->p[1] == ':')) {
        e->p += 2;
        return (int64_t)expr_reverse32((uint32_t)expr_reverse(e));
    }
    return expr_shift(e);
}

/* Resolve an integer operand: a full expression (literal / define / label /
 * arithmetic). Truncated to 32 bits. */
static bool resolve_uint(const asm_ctx_t *ctx, const char *s, uint32_t *out)
{
    expr_t e = {ctx, s, true};
    int64_t v = expr_reverse(&e);
    expr_skip_ws(&e);
    if (!e.ok || (*e.p != '\0')) {
        return false;
    }
    *out = (uint32_t)((uint64_t)v & 0xFFFFFFFFU);
    return true;
}

/* Join tokens [from, n) with single spaces into `buf` for expression parsing
 * (operands may span tokens when operators are spaced). Returns false if the
 * buffer is too small or the range is empty. */
static bool join_tokens(char *const tok[], uint8_t from, uint8_t n, char *buf, size_t bufsz)
{
    size_t pos = 0;
    for (uint8_t i = from; i < n; i++) {
        size_t tl = strlen(tok[i]);
        if ((pos + tl + 2U) >= bufsz) {
            return false;
        }
        if (pos > 0U) {
            buf[pos] = ' ';
            pos++;
        }
        (void)memcpy(&buf[pos], tok[i], tl);
        pos += tl;
    }
    buf[pos] = '\0';
    return pos > 0U;
}

/* Resolve an expression spanning tokens [from, n). */
static bool resolve_uint_join(const asm_ctx_t *ctx, char *const tok[], uint8_t from, uint8_t n,
                              uint32_t *out)
{
    char buf[MAX_LINE];
    if (!join_tokens(tok, from, n, buf, sizeof(buf))) {
        return false;
    }
    return resolve_uint(ctx, buf, out);
}

/* Record a `.define`d symbol; a later definition of the same name overrides. */
static bool add_define(asm_ctx_t *ctx, const char *name, uint32_t value)
{
    for (uint8_t i = 0; i < ctx->define_count; i++) {
        if (strcmp(ctx->defines[i].name, name) == 0) {
            ctx->defines[i].value = value;
            return true;
        }
    }
    if (ctx->define_count >= MAX_DEFINES) {
        return false;
    }
    define_t *d = &ctx->defines[ctx->define_count];
    (void)snprintf(d->name, sizeof(d->name), "%s", name);
    d->value = value;
    ctx->define_count++;
    return true;
}

/* ── Operand encoders ──────────────────────────────────────────────────────── */

/* IN/MOV source name → field; returns -1 on unknown. */
static int src_field(const char *s)
{
    if (ieq(s, "pins")) {
        return PIO_SRC_PINS;
    }
    if (ieq(s, "x")) {
        return PIO_SRC_X;
    }
    if (ieq(s, "y")) {
        return PIO_SRC_Y;
    }
    if (ieq(s, "null")) {
        return PIO_SRC_NULL;
    }
    if (ieq(s, "status")) {
        return PIO_SRC_STATUS;
    }
    if (ieq(s, "isr")) {
        return PIO_SRC_ISR;
    }
    if (ieq(s, "osr")) {
        return PIO_SRC_OSR;
    }
    return -1;
}

/* OUT destination name → field. */
static int out_dest_field(const char *s)
{
    if (ieq(s, "pins")) {
        return 0;
    }
    if (ieq(s, "x")) {
        return 1;
    }
    if (ieq(s, "y")) {
        return 2;
    }
    if (ieq(s, "null")) {
        return 3;
    }
    if (ieq(s, "pindirs")) {
        return 4;
    }
    if (ieq(s, "pc")) {
        return 5;
    }
    if (ieq(s, "isr")) {
        return 6;
    }
    if (ieq(s, "exec")) {
        return 7;
    }
    return -1;
}

/* MOV destination name → field. (MOV's table differs from OUT/SET: pindirs=3.) */
static int mov_dest_field(const char *s)
{
    if (ieq(s, "pins")) {
        return 0;
    }
    if (ieq(s, "x")) {
        return 1;
    }
    if (ieq(s, "y")) {
        return 2;
    }
#if PIO_SIM_HAS_MOV_PINDIRS
    if (ieq(s, "pindirs")) {
        return 3;
    }
#endif
    if (ieq(s, "exec")) {
        return 4;
    }
    if (ieq(s, "pc")) {
        return 5;
    }
    if (ieq(s, "isr")) {
        return 6;
    }
    if (ieq(s, "osr")) {
        return 7;
    }
    return -1;
}

/* SET destination name → field. */
static int set_dest_field(const char *s)
{
    if (ieq(s, "pins")) {
        return 0;
    }
    if (ieq(s, "x")) {
        return 1;
    }
    if (ieq(s, "y")) {
        return 2;
    }
    if (ieq(s, "pindirs")) {
        return 4;
    }
    return -1;
}

/* Strip a leading invert/reverse marker from a MOV source token. */
static uint8_t mov_op_of(char **src)
{
    char *s = *src;
    if ((s[0] == '!') || (s[0] == '~')) {
        *src = &s[1];
        return PIO_MOV_INVERT;
    }
    if ((s[0] == ':') && (s[1] == ':')) {
        *src = &s[2];
        return PIO_MOV_REVERSE;
    }
    return PIO_MOV_NONE;
}

/*
 * Pull trailing "side <n>" and "[<delay>]" tokens off the token list, in any
 * order. Updates *n to the remaining count. Stores results in *side / *side_set
 * and *delay.
 */
static bool extract_side_delay(asm_ctx_t *ctx, char *tok[], uint8_t *n, uint32_t *side,
                               bool *has_side, uint32_t *delay)
{
    *has_side = false;
    *side = 0;
    *delay = 0;
    bool changed = true;
    while (changed && (*n > 0U)) {
        changed = false;
        /* delay form "[k]" as a single token */
        char *last = tok[*n - 1U];
        size_t len = strlen(last);
        if ((len >= 2U) && ((last[0] == '[') && (last[len - 1U] == ']'))) {
            last[len - 1U] = '\0';
            uint32_t d;
            if (!resolve_uint(ctx, &last[1], &d)) {
                pa_set_error(ctx, "bad delay value");
                return false;
            }
            *delay = d;
            (*n)--;
            changed = true;
            continue;
        }
        /* side form "side <n>" as two tokens */
        if ((*n >= 2U) && ieq(tok[*n - 2U], "side")) {
            uint32_t sv;
            if (!resolve_uint(ctx, tok[*n - 1U], &sv)) {
                pa_set_error(ctx, "bad side-set value");
                return false;
            }
            *side = sv;
            *has_side = true;
            *n = (uint8_t)(*n - 2U);
            changed = true;
            continue;
        }
    }
    return true;
}

/* Combine the decoded delay/side-set into the [12:8] field. */
static bool pack_delay_sideset(asm_ctx_t *ctx, uint32_t delay, uint32_t side, bool has_side,
                               uint16_t *field)
{
    pio_program_t *p = ctx->out;
    /* Side-set occupies at most 5 bits; clamp so the shifts below are provably
     * in range (and malformed .side_set widths can't overflow). */
    uint8_t ss_total = (p->sideset_bits > 5U) ? 5U : p->sideset_bits;
    uint8_t delay_bits = (uint8_t)((5U - ss_total) & 7U);
    uint32_t max_delay = ((uint32_t)1U << delay_bits) - 1U;
    if (delay > max_delay) {
        pa_set_error(ctx, "delay too large for side-set width");
        return false;
    }
    uint16_t v = (uint16_t)(delay & max_delay);
    if (ss_total > 0U) {
        if (p->sideset_opt) {
            if (has_side) {
                uint8_t data_bits = (uint8_t)((ss_total - 1U) & 7U);
                uint32_t max_side = ((uint32_t)1U << data_bits) - 1U;
                if (side > max_side) {
                    pa_set_error(ctx, "side-set value out of range");
                    return false;
                }
                v |= (uint16_t)0x10U; /* enable bit (side-set) */
                v |= (uint16_t)(side << delay_bits);
            }
            /* opt + no side token → enable bit stays 0 */
        } else {
            uint32_t max_side = ((uint32_t)1U << ss_total) - 1U;
            if (!has_side) {
                pa_set_error(ctx, "instruction missing required side-set");
                return false;
            }
            if (side > max_side) {
                pa_set_error(ctx, "side-set value out of range");
                return false;
            }
            v |= (uint16_t)(side << delay_bits);
        }
    } else if (has_side) {
        pa_set_error(ctx, "side-set used but .side_set not declared");
        return false;
    } else {
        /* No side-set value to encode. */
    }
    *field = v;
    return true;
}

/* ── Instruction assembly ──────────────────────────────────────────────────── */

/* Each mnemonic encoder fills *base with the opcode+operand word (no
 * delay/side-set field) and returns true, or sets an error and returns false. */

static bool enc_jmp(asm_ctx_t *ctx, char *tok[], uint8_t n, uint16_t *base)
{
    if (n < 2U) {
        pa_set_error(ctx, "jmp needs a target");
        return false;
    }
    uint8_t cond = PIO_COND_ALWAYS;
    uint8_t target_from = 1U;
    const char *c = tok[1];
    bool has_cond = true;
    if (ieq(c, "!x")) {
        cond = PIO_COND_NOTX;
    } else if (ieq(c, "x--")) {
        cond = PIO_COND_XDEC;
    } else if (ieq(c, "!y")) {
        cond = PIO_COND_NOTY;
    } else if (ieq(c, "y--")) {
        cond = PIO_COND_YDEC;
    } else if (ieq(c, "x!=y")) {
        cond = PIO_COND_XNEY;
    } else if (ieq(c, "pin")) {
        cond = PIO_COND_PIN;
    } else if (ieq(c, "!osre")) {
        cond = PIO_COND_NOTOSRE;
    } else {
        has_cond = false; /* tok[1] is the target expression */
    }
    if (has_cond) {
        target_from = 2U;
        if (n < 3U) {
            pa_set_error(ctx, "jmp condition needs a target");
            return false;
        }
    }
    uint32_t addr;
    if (!resolve_uint_join(ctx, tok, target_from, n, &addr)) {
        pa_set_error(ctx, "unknown jmp target/label");
        return false;
    }
    *base = pio_sim_encode_jmp(cond, (uint8_t)addr);
    return true;
}

static bool is_wait_source(const char *t)
{
    if (ieq(t, "gpio") || ieq(t, "pin") || ieq(t, "irq")) {
        return true;
    }
#if PIO_SIM_HAS_WAIT_JMPPIN
    if (ieq(t, "jmppin")) {
        return true;
    }
#endif
    return false;
}

static bool enc_wait(asm_ctx_t *ctx, char *tok[], uint8_t n, uint16_t *base)
{
    if (n < 2U) {
        pa_set_error(ctx, "wait needs <src> <index>");
        return false;
    }
    /* Polarity is optional (pioasm defaults it to 1): `wait <pol> <src> …` or
     * `wait <src> …`. */
    uint32_t pol = 1;
    uint8_t si; /* token index of the source keyword */
    if (is_wait_source(tok[1])) {
        si = 1U;
    } else {
        if (!resolve_uint(ctx, tok[1], &pol)) {
            pa_set_error(ctx, "bad wait polarity");
            return false;
        }
        si = 2U;
    }
    if (si >= n) {
        pa_set_error(ctx, "wait needs a source");
        return false;
    }
    const char *srctok = tok[si];
    uint8_t source;
    if (ieq(srctok, "gpio")) {
        source = PIO_WAIT_GPIO;
    } else if (ieq(srctok, "pin")) {
        source = PIO_WAIT_PIN;
    } else if (ieq(srctok, "irq")) {
        source = PIO_WAIT_IRQ;
#if PIO_SIM_HAS_WAIT_JMPPIN
    } else if (ieq(srctok, "jmppin")) {
        source = PIO_WAIT_JMPPIN;
#endif
    } else {
        pa_set_error(ctx, "bad wait source");
        return false;
    }

#if PIO_SIM_HAS_WAIT_JMPPIN
    if (source == PIO_WAIT_JMPPIN) {
        /* `jmppin` (index 0) or `jmppin + <expr>`. */
        uint32_t idx = 0;
        if (n > (uint8_t)(si + 1U)) {
            uint8_t from = (uint8_t)(si + 1U);
            if (ieq(tok[from], "+")) {
                from++;
            }
            if ((from >= n) || !resolve_uint_join(ctx, tok, from, n, &idx)) {
                pa_set_error(ctx, "bad jmppin index");
                return false;
            }
        }
        *base = pio_sim_encode_wait_jmppin((uint8_t)pol, (uint8_t)idx);
        return true;
    }
#endif

    /* For irq: optional `prev`/`next` precede the index; optional `rel` trails. */
    uint8_t idx_from = (uint8_t)(si + 1U);
    uint8_t idx_to = n;
    bool rel = false;
    bool prev = false;
    bool next = false;
    if (source == PIO_WAIT_IRQ) {
#if PIO_SIM_HAS_IRQ_PREVNEXT
        if ((idx_from < n) && ieq(tok[idx_from], "prev")) {
            prev = true;
            idx_from++;
        } else if ((idx_from < n) && ieq(tok[idx_from], "next")) {
            next = true;
            idx_from++;
        } else {
            /* absolute or rel */
        }
#endif
        if ((n > idx_from) && ieq(tok[n - 1U], "rel")) {
            rel = true;
            idx_to = (uint8_t)(n - 1U);
        }
    }
    if (idx_from >= idx_to) {
        pa_set_error(ctx, "wait needs an index");
        return false;
    }
    uint32_t idx;
    if (!resolve_uint_join(ctx, tok, idx_from, idx_to, &idx)) {
        pa_set_error(ctx, "bad wait index");
        return false;
    }
    if (rel) {
        *base = pio_sim_encode_wait_irq_rel((uint8_t)pol, (uint8_t)idx);
        return true;
    }
    uint8_t field = (uint8_t)(idx & 0x1FU);
#if PIO_SIM_HAS_IRQ_PREVNEXT
    if (prev) {
        field = (uint8_t)((idx & 0x7U) | PIO_IRQ_PREV);
    } else if (next) {
        field = (uint8_t)((idx & 0x7U) | PIO_IRQ_NEXT);
    } else {
        /* absolute index */
    }
#else
    (void)prev;
    (void)next;
#endif
    *base = pio_sim_encode_wait((uint8_t)pol, source, field);
    return true;
}

static bool enc_in(asm_ctx_t *ctx, char *tok[], uint8_t n, uint16_t *base)
{
    if (n < 3U) {
        pa_set_error(ctx, "in needs <src>, <count>");
        return false;
    }
    int src = src_field(tok[1]);
    if (src == (int)PIO_SRC_STATUS) {
        /* IN source encoding 101 is reserved on hardware (STATUS is a MOV-only
         * source); real pioasm rejects it too. */
        pa_set_error(ctx, "status is not a valid in source");
        return false;
    }
    uint32_t cnt;
    if ((src < 0) || !resolve_uint_join(ctx, tok, 2, n, &cnt)) {
        pa_set_error(ctx, "bad in operands");
        return false;
    }
    *base = pio_sim_encode_in((uint8_t)src, (uint8_t)(cnt & 0x1FU));
    return true;
}

static bool enc_out(asm_ctx_t *ctx, char *tok[], uint8_t n, uint16_t *base)
{
    if (n < 3U) {
        pa_set_error(ctx, "out needs <dest>, <count>");
        return false;
    }
    int dest = out_dest_field(tok[1]);
    uint32_t cnt;
    if ((dest < 0) || !resolve_uint_join(ctx, tok, 2, n, &cnt)) {
        pa_set_error(ctx, "bad out operands");
        return false;
    }
    *base = pio_sim_encode_out((uint8_t)dest, (uint8_t)(cnt & 0x1FU));
    return true;
}

static bool enc_push(asm_ctx_t *ctx, char *tok[], uint8_t n, uint16_t *base)
{
    bool if_full = false;
    bool block = true;
    for (uint8_t i = 1; i < n; i++) {
        if (ieq(tok[i], "iffull")) {
            if_full = true;
        } else if (ieq(tok[i], "block")) {
            block = true;
        } else if (ieq(tok[i], "noblock")) {
            block = false;
        } else {
            pa_set_error(ctx, "bad push qualifier");
            return false;
        }
    }
    *base = pio_sim_encode_push(if_full, block);
    return true;
}

static bool enc_pull(asm_ctx_t *ctx, char *tok[], uint8_t n, uint16_t *base)
{
    bool if_empty = false;
    bool block = true;
    for (uint8_t i = 1; i < n; i++) {
        if (ieq(tok[i], "ifempty")) {
            if_empty = true;
        } else if (ieq(tok[i], "block")) {
            block = true;
        } else if (ieq(tok[i], "noblock")) {
            block = false;
        } else {
            pa_set_error(ctx, "bad pull qualifier");
            return false;
        }
    }
    *base = pio_sim_encode_pull(if_empty, block);
    return true;
}

#if PIO_SIM_HAS_RXFIFO_MOV
/* Match an "rxfifo[<n|y>]" operand. Returns true on a match, setting *is_y when
 * the index is the scratch register Y, else *idx to the literal index. */
static bool parse_rxfifo(const char *tok, bool *is_y, uint8_t *idx)
{
    static const char pfx[] = "rxfifo[";
    const char *s = tok;
    for (uint8_t i = 0; pfx[i] != '\0'; i++) {
        if (lower_ascii(*s) != pfx[i]) {
            return false;
        }
        s++;
    }
    if ((*s == 'y') || (*s == 'Y')) {
        *is_y = true;
        s++;
    } else if ((*s >= '0') && (*s <= '9')) {
        *is_y = false;
        *idx = (uint8_t)(*s - '0');
        s++;
    } else {
        return false;
    }
    return (*s == ']') && (s[1] == '\0');
}
#endif /* PIO_SIM_HAS_RXFIFO_MOV */

static bool enc_mov(asm_ctx_t *ctx, char *tok[], uint8_t n, uint16_t *base)
{
    if (n < 3U) {
        pa_set_error(ctx, "mov needs <dest>, <src>");
        return false;
    }
#if PIO_SIM_HAS_RXFIFO_MOV
    /* RP2350 indexed RX-FIFO moves: `mov rxfifo[<n|y>], isr` and
     * `mov osr, rxfifo[<n|y>]`. */
    bool ry = false;
    uint8_t ridx = 0;
    if (parse_rxfifo(tok[1], &ry, &ridx) && ieq(tok[2], "isr")) {
        *base = ry ? pio_sim_encode_mov_to_rxfifo_y() : pio_sim_encode_mov_to_rxfifo(ridx);
        return true;
    }
    if (ieq(tok[1], "osr") && parse_rxfifo(tok[2], &ry, &ridx)) {
        *base = ry ? pio_sim_encode_mov_from_rxfifo_y() : pio_sim_encode_mov_from_rxfifo(ridx);
        return true;
    }
#endif
    int dest = mov_dest_field(tok[1]);
    char *srctok = tok[2];
    uint8_t op = mov_op_of(&srctok);
    int src = src_field(srctok);
    if ((dest < 0) || (src < 0)) {
        pa_set_error(ctx, "bad mov operands");
        return false;
    }
    *base = pio_sim_encode_mov((uint8_t)dest, op, (uint8_t)src);
    return true;
}

static bool enc_irq(asm_ctx_t *ctx, char *tok[], uint8_t n, uint16_t *base)
{
    bool clear = false;
    bool wait = false;
    bool rel = false;
    bool prev = false;
    bool next = false;
    uint8_t idx_from = 0;
    uint8_t idx_to = 0;
    bool have_idx = false;
    for (uint8_t i = 1; i < n; i++) {
        if (ieq(tok[i], "nowait") || ieq(tok[i], "set")) {
            wait = false;
        } else if (ieq(tok[i], "wait")) {
            wait = true;
        } else if (ieq(tok[i], "clear")) {
            clear = true;
        } else if (ieq(tok[i], "rel")) {
            rel = true;
#if PIO_SIM_HAS_IRQ_PREVNEXT
        } else if (ieq(tok[i], "prev")) {
            prev = true;
        } else if (ieq(tok[i], "next")) {
            next = true;
#endif
        } else {
            if (!have_idx) {
                idx_from = i;
                have_idx = true;
            }
            idx_to = (uint8_t)(i + 1U); /* index expression spans the non-keyword run */
        }
    }
    if (!have_idx) {
        pa_set_error(ctx, "irq needs an index");
        return false;
    }
    uint32_t idx;
    if (!resolve_uint_join(ctx, tok, idx_from, idx_to, &idx)) {
        pa_set_error(ctx, "bad irq index");
        return false;
    }
    if (rel && (prev || next)) {
        pa_set_error(ctx, "irq rel cannot combine with prev/next");
        return false;
    }
    if (rel) {
        *base = pio_sim_encode_irq_rel(clear, wait, (uint8_t)idx);
        return true;
    }
    uint8_t field = (uint8_t)(idx & 0x1FU);
#if PIO_SIM_HAS_IRQ_PREVNEXT
    if (prev) {
        field = (uint8_t)((idx & 0x7U) | PIO_IRQ_PREV);
    } else if (next) {
        field = (uint8_t)((idx & 0x7U) | PIO_IRQ_NEXT);
    } else {
        /* absolute index */
    }
#else
    (void)prev;
    (void)next;
#endif
    *base = pio_sim_encode_irq(clear, wait, field);
    return true;
}

static bool enc_set(asm_ctx_t *ctx, char *tok[], uint8_t n, uint16_t *base)
{
    if (n < 3U) {
        pa_set_error(ctx, "set needs <dest>, <value>");
        return false;
    }
    int dest = set_dest_field(tok[1]);
    uint32_t val;
    if ((dest < 0) || !resolve_uint_join(ctx, tok, 2, n, &val)) {
        pa_set_error(ctx, "bad set operands");
        return false;
    }
    *base = pio_sim_encode_set((uint8_t)dest, (uint8_t)(val & 0x1FU));
    return true;
}

/* Encode the opcode word for the mnemonic in tok[0]. */
static bool encode_mnemonic(asm_ctx_t *ctx, char *tok[], uint8_t n, uint16_t *base)
{
    const char *m = tok[0];
    if (ieq(m, "nop")) {
        *base = pio_sim_encode_nop();
        return true;
    }
    if (ieq(m, "jmp")) {
        return enc_jmp(ctx, tok, n, base);
    }
    if (ieq(m, "wait")) {
        return enc_wait(ctx, tok, n, base);
    }
    if (ieq(m, "in")) {
        return enc_in(ctx, tok, n, base);
    }
    if (ieq(m, "out")) {
        return enc_out(ctx, tok, n, base);
    }
    if (ieq(m, "push")) {
        return enc_push(ctx, tok, n, base);
    }
    if (ieq(m, "pull")) {
        return enc_pull(ctx, tok, n, base);
    }
    if (ieq(m, "mov")) {
        return enc_mov(ctx, tok, n, base);
    }
    if (ieq(m, "irq")) {
        return enc_irq(ctx, tok, n, base);
    }
    if (ieq(m, "set")) {
        return enc_set(ctx, tok, n, base);
    }
    pa_set_error(ctx, "unknown mnemonic");
    return false;
}

static bool assemble_insn(asm_ctx_t *ctx, char *tok[], uint8_t n, uint16_t *out)
{
    uint32_t delay = 0;
    uint32_t side = 0;
    bool has_side = false;
    if (!extract_side_delay(ctx, tok, &n, &side, &has_side, &delay)) {
        return false;
    }
    uint16_t base = 0;
    if (!encode_mnemonic(ctx, tok, n, &base)) {
        return false;
    }
    uint16_t ds = 0;
    if (!pack_delay_sideset(ctx, delay, side, has_side, &ds)) {
        return false;
    }
    /* ds is the 5-bit [12:8] delay/side-set field; place it at bit 8. */
    *out = (uint16_t)(base | (uint16_t)(ds << 8U));
    return true;
}

/* ── Line classification ───────────────────────────────────────────────────── */

/* Strip a ';' comment and trailing/leading whitespace; returns the cleaned
 * string (in place). */
/* Overwrite `/ * ... * /` block-comment spans with spaces, carrying the open
 * state across lines via *in_block. */
static void strip_block_comments(char *line, bool *in_block)
{
    char *p = line;
    while (*p != '\0') {
        if (*in_block) {
            if ((p[0] == '*') && (p[1] == '/')) {
                p[0] = ' ';
                p[1] = ' ';
                p = &p[2];
                *in_block = false;
            } else {
                *p = ' ';
                p++;
            }
        } else if ((p[0] == '/') && (p[1] == '*')) {
            p[0] = ' ';
            p[1] = ' ';
            p = &p[2];
            *in_block = true;
        } else {
            p++;
        }
    }
}

static char *clean_line(char *line)
{
    /* Cut at the earliest of a ';' or '//' line comment. */
    char *semi = strchr(line, ';');
    char *slash = strstr(line, "//");
    char *cut = semi;
    if ((slash != NULL) && ((cut == NULL) || (slash < cut))) {
        cut = slash;
    }
    if (cut != NULL) {
        *cut = '\0';
    }
    /* trim leading */
    char *s = line;
    while ((*s == ' ') || (*s == '\t') || (*s == '\r') || (*s == '\n')) {
        s++;
    }
    /* trim trailing */
    size_t len = strlen(s);
    while ((len > 0U) && ((s[len - 1U] == ' ') || (s[len - 1U] == '\t') || (s[len - 1U] == '\r') ||
                          (s[len - 1U] == '\n'))) {
        --len;
        s[len] = '\0';
    }
    return s;
}

/* ── Two-pass assembly ─────────────────────────────────────────────────────── */

/* Mutable per-pass parser state. */
typedef struct {
    asm_ctx_t *ctx;
    const char *name; /* selected program, or NULL for first */
    int pass;         /* 1 = collect, 2 = emit */
    bool in_program;
    bool done_selected;
    bool in_verbatim;
    bool in_block_comment;
    bool seen_any_program;
    uint8_t idx; /* instruction index within the program */
} pa_parse_state_t;

typedef enum {
    LINE_OK,    /* processed, continue */
    LINE_DONE,  /* stop scanning (next program reached) */
    LINE_ERROR, /* error set on ctx */
} line_result_t;

/* Tokenize a copy of `line` (preserving the original). Returns token count and
 * sets *truncated if the line had more than MAX_TOKENS tokens. */
static uint8_t tokenize_copy(const char *line, char tmp[MAX_LINE], char *tok[MAX_TOKENS],
                             bool *truncated)
{
    int written = snprintf(tmp, MAX_LINE, "%s", line);
    (void)written;
    for (uint8_t i = 0; i < MAX_TOKENS; i++) {
        tok[i] = NULL;
    }
    return tokenize(tmp, tok, MAX_TOKENS, truncated);
}

static void parse_side_set(pa_parse_state_t *ps, char *tok[], uint8_t nt, uint32_t bits)
{
    pio_program_t *out = ps->ctx->out;
    bool opt = false;
    bool pindirs = false;
    for (uint8_t i = 2; i < nt; i++) {
        if (ieq(tok[i], "opt")) {
            opt = true;
        } else if (ieq(tok[i], "pindirs")) {
            pindirs = true;
        } else {
            /* Unrecognised modifier ignored. */
        }
    }
    out->sideset_bits = (uint8_t)(bits + (opt ? 1U : 0U));
    out->sideset_opt = opt;
    out->sideset_pindirs = pindirs;
}

/* Parse a non-negative decimal/float for .clock_div (e.g. "2.5"). */
static bool parse_double(const char *s, double *out)
{
    char *end = NULL;
    double v = strtod(s, &end);
    if ((end == s) || (*end != '\0') || (v < 0.0)) {
        return false;
    }
    *out = v;
    return true;
}

/* `.fifo <txrx|tx|rx|txput|txget|putget>` → FIFO-join mode. */
static bool parse_fifo_config(const char *t, pio_fifo_join_t *out)
{
    if (ieq(t, "txrx")) {
        *out = PIO_FIFO_JOIN_NONE;
        return true;
    }
    if (ieq(t, "tx")) {
        *out = PIO_FIFO_JOIN_TX;
        return true;
    }
    if (ieq(t, "rx")) {
        *out = PIO_FIFO_JOIN_RX;
        return true;
    }
#if PIO_SIM_HAS_RXFIFO_MOV
    if (ieq(t, "txput") || ieq(t, "putget")) {
        *out = PIO_FIFO_JOIN_RX_PUT;
        return true;
    }
    if (ieq(t, "txget")) {
        *out = PIO_FIFO_JOIN_RX_GET;
        return true;
    }
#endif
    return false;
}

/* `.in/.out <count> [left|right] [auto|manual] [threshold]`. */
static bool parse_shift_dir_cfg(pa_parse_state_t *ps, char *tok[], uint8_t nt, bool is_in)
{
    uint32_t count = 0;
    if ((nt < 2U) || !resolve_uint(ps->ctx, tok[1], &count)) {
        return false;
    }
    pio_shift_dir_t dir = PIO_SHIFT_RIGHT; /* pioasm default */
    bool autoshift = false;
    uint32_t threshold = 32;
    for (uint8_t i = 2; i < nt; i++) {
        if (ieq(tok[i], "left")) {
            dir = PIO_SHIFT_LEFT;
        } else if (ieq(tok[i], "right")) {
            dir = PIO_SHIFT_RIGHT;
        } else if (ieq(tok[i], "auto")) {
            autoshift = true;
        } else if (ieq(tok[i], "manual")) {
            autoshift = false;
        } else if (!resolve_uint(ps->ctx, tok[i], &threshold)) {
            return false;
        } else {
            /* threshold captured */
        }
    }
    pio_program_t *out = ps->ctx->out;
    if (is_in) {
        out->in_cfg.set = true;
        out->in_cfg.count = (uint8_t)count;
        out->in_cfg.dir = dir;
        out->in_cfg.autoshift = autoshift;
        out->in_cfg.threshold = (uint8_t)threshold;
    } else {
        out->out_cfg.set = true;
        out->out_cfg.count = (uint8_t)count;
        out->out_cfg.dir = dir;
        out->out_cfg.autoshift = autoshift;
        out->out_cfg.threshold = (uint8_t)threshold;
    }
    return true;
}

/* `.mov_status <txfifo < N | rxfifo < N | irq [next|prev] set N>`. The number is
 * the last token; `<`/`set`/`next`/`prev` tokens are positional markers. */
static bool parse_mov_status(pa_parse_state_t *ps, char *tok[], uint8_t nt)
{
    if (nt < 2U) {
        return false;
    }
    pio_program_t *out = ps->ctx->out;
    if (ieq(tok[1], "txfifo")) {
        out->mov_status_sel = PIO_STATUS_TX_LEVEL;
    } else if (ieq(tok[1], "rxfifo")) {
        out->mov_status_sel = PIO_STATUS_RX_LEVEL;
#if PIO_SIM_HAS_IRQ_STATUS
    } else if (ieq(tok[1], "irq")) {
        out->mov_status_sel = PIO_STATUS_IRQ_SET;
#endif
    } else {
        return false;
    }
    uint32_t n = 0;
    if (!resolve_uint(ps->ctx, tok[nt - 1U], &n)) {
        return false;
    }
    out->mov_status_n = (uint8_t)n;
    out->has_mov_status = true;
    return true;
}

static line_result_t handle_directive(pa_parse_state_t *ps, const char *line)
{
    char tmp[MAX_LINE];
    char *tok[MAX_TOKENS];
    bool truncated = false;
    uint8_t nt = tokenize_copy(line, tmp, tok, &truncated);
    if (truncated) {
        pa_set_error(ps->ctx, "directive has too many tokens");
        return LINE_ERROR;
    }
    if (nt == 0U) {
        return LINE_OK;
    }
    pio_program_t *out = ps->ctx->out;

    if (ieq(tok[0], ".program")) {
        if (ps->seen_any_program && ps->in_program) {
            return LINE_DONE; /* next program reached — stop */
        }
        ps->seen_any_program = true;
        ps->in_program = (ps->name == NULL) || ((nt >= 2U) && (strcmp(tok[1], ps->name) == 0));
        if (ps->in_program && (ps->pass == 1) && (nt >= 2U)) {
            (void)snprintf(out->name, sizeof(out->name), "%s", tok[1]);
        }
        ps->idx = 0;
        return LINE_OK;
    }
    /* `.define [PUBLIC] <NAME> <value>` — handled before the in_program gate
     * because top-level defines precede any .program. Scoping matches pioasm:
     * top-level and PUBLIC defines are global; a non-public define inside a
     * program body is local to it, so one from a *non-selected* program must
     * not leak into the selected program's symbol table. */
    if (ieq(tok[0], ".define")) {
        if (ps->pass == 1) {
            uint8_t ni = ((nt >= 2U) && ieq(tok[1], "public")) ? 2U : 1U;
            if ((ni == 1U) && ps->seen_any_program && !ps->in_program) {
                return LINE_OK; /* non-public define in another program: skip */
            }
            uint32_t val = 0;
            if ((nt < (uint8_t)(ni + 2U)) ||
                !resolve_uint_join(ps->ctx, tok, (uint8_t)(ni + 1U), nt, &val)) {
                pa_set_error(ps->ctx, "bad .define");
                return LINE_ERROR;
            }
            if (!add_define(ps->ctx, tok[ni], val)) {
                pa_set_error(ps->ctx, "too many .define symbols");
                return LINE_ERROR;
            }
        }
        return LINE_OK;
    }
    if (!ps->in_program) {
        return LINE_OK;
    }
    if (ieq(tok[0], ".side_set")) {
        if (ps->pass == 1) {
            uint32_t bits = 0;
            if ((nt < 2U) || !resolve_uint(ps->ctx, tok[1], &bits)) {
                pa_set_error(ps->ctx, "bad .side_set");
                return LINE_ERROR;
            }
            parse_side_set(ps, tok, nt, bits);
        }
    } else if (ieq(tok[0], ".wrap_target")) {
        if (ps->pass == 1) {
            out->wrap_bottom = ps->idx;
        }
    } else if (ieq(tok[0], ".wrap")) {
        if (ps->pass == 1) {
            out->wrap_top = (ps->idx > 0U) ? (uint8_t)(ps->idx - 1U) : 0U;
        }
    } else if (ieq(tok[0], ".word")) {
        /* Emits a raw instruction word, so it occupies a slot in both passes. */
        if (ps->idx >= PIO_ASM_MAX_INSNS) {
            pa_set_error(ps->ctx, "program exceeds 32 instructions");
            return LINE_ERROR;
        }
        uint32_t w = 0;
        if ((nt < 2U) || !resolve_uint_join(ps->ctx, tok, 1, nt, &w)) {
            pa_set_error(ps->ctx, "bad .word");
            return LINE_ERROR;
        }
        if (ps->pass == 2) {
            out->insns[ps->idx] = (uint16_t)w;
        }
        ps->idx++;
    } else if (ps->pass != 1) {
        /* Remaining directives are pass-1 metadata only. */
        return LINE_OK;
    } else if (ieq(tok[0], ".origin")) {
        uint32_t v = 0;
        if ((nt < 2U) || !resolve_uint(ps->ctx, tok[1], &v)) {
            pa_set_error(ps->ctx, "bad .origin");
            return LINE_ERROR;
        }
        out->has_origin = true;
        out->origin = (uint8_t)v;
    } else if (ieq(tok[0], ".pio_version")) {
        if (nt < 2U) {
            pa_set_error(ps->ctx, "bad .pio_version");
            return LINE_ERROR;
        }
        if (ieq(tok[1], "rp2040")) {
            out->pio_version = 0;
        } else if (ieq(tok[1], "rp2350")) {
            out->pio_version = 1;
        } else {
            uint32_t v = 0;
            if (!resolve_uint(ps->ctx, tok[1], &v)) {
                pa_set_error(ps->ctx, "bad .pio_version");
                return LINE_ERROR;
            }
            out->pio_version = (int)v;
        }
    } else if (ieq(tok[0], ".clock_div")) {
        double d = 0.0;
        if ((nt < 2U) || !parse_double(tok[1], &d)) {
            pa_set_error(ps->ctx, "bad .clock_div");
            return LINE_ERROR;
        }
        out->has_clock_div = true;
        out->clock_div = d;
    } else if (ieq(tok[0], ".fifo")) {
        if ((nt < 2U) || !parse_fifo_config(tok[1], &out->fifo_join)) {
            pa_set_error(ps->ctx, "bad .fifo");
            return LINE_ERROR;
        }
        out->has_fifo_join = true;
    } else if (ieq(tok[0], ".mov_status")) {
        if (!parse_mov_status(ps, tok, nt)) {
            pa_set_error(ps->ctx, "bad .mov_status");
            return LINE_ERROR;
        }
    } else if (ieq(tok[0], ".in")) {
        if (!parse_shift_dir_cfg(ps, tok, nt, true)) {
            pa_set_error(ps->ctx, "bad .in");
            return LINE_ERROR;
        }
    } else if (ieq(tok[0], ".out")) {
        if (!parse_shift_dir_cfg(ps, tok, nt, false)) {
            pa_set_error(ps->ctx, "bad .out");
            return LINE_ERROR;
        }
    } else if (ieq(tok[0], ".set")) {
        uint32_t v = 0;
        if ((nt < 2U) || !resolve_uint(ps->ctx, tok[1], &v)) {
            pa_set_error(ps->ctx, "bad .set");
            return LINE_ERROR;
        }
        out->set_cfg.set = true;
        out->set_cfg.count = (uint8_t)v;
    } else if (ieq(tok[0], ".lang_opt")) {
        /* Language-specific output option: parsed and ignored. */
    } else {
        /* Unknown directive: ignore for forward compatibility. */
    }
    return LINE_OK;
}

/* Strip a leading "public " label prefix. */
static char *strip_public(char *line)
{
    if ((strncmp(line, "public ", 7) == 0) || (strncmp(line, "PUBLIC ", 7) == 0)) {
        char *q = &line[7];
        while ((*q == ' ') || (*q == '\t')) {
            q++;
        }
        if (strchr(q, ':') != NULL) {
            return q;
        }
    }
    return line;
}

/*
 * If `line` begins with a "label:" declaration, record it (pass 1) and advance
 * `*line_io` past it. Returns true if the line was label-only (no instruction).
 */
static bool handle_label(pa_parse_state_t *ps, char **line_io)
{
    char *orig = *line_io;
    char *line = strip_public(orig);
    bool is_public = (line != orig);
    *line_io = line;

    char *colon = strchr(line, ':');
    const char *id = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_";
    size_t namelen = strspn(line, id);
    if ((colon == NULL) || (colon != &line[namelen])) {
        return false;
    }
    asm_ctx_t *ctx = ps->ctx;
    if ((ps->pass == 1) && (ctx->label_count < MAX_LABELS)) {
        if (namelen < sizeof(ctx->labels[0].name)) {
            (void)memcpy(ctx->labels[ctx->label_count].name, line, namelen);
            ctx->labels[ctx->label_count].name[namelen] = '\0';
            ctx->labels[ctx->label_count].index = ps->idx;
            ctx->labels[ctx->label_count].is_public = is_public;
            ctx->label_count++;
        }
    }
    char *rest = &colon[1];
    while ((*rest == ' ') || (*rest == '\t')) {
        rest++;
    }
    *line_io = rest;
    return *rest == '\0';
}

static line_result_t handle_instruction(pa_parse_state_t *ps, const char *line)
{
    if (ps->idx >= PIO_ASM_MAX_INSNS) {
        pa_set_error(ps->ctx, "program exceeds 32 instructions");
        return LINE_ERROR;
    }
    if (ps->pass == 2) {
        char tmp[MAX_LINE];
        char *tok[MAX_TOKENS];
        bool truncated = false;
        uint8_t nt = tokenize_copy(line, tmp, tok, &truncated);
        if (truncated) {
            pa_set_error(ps->ctx, "instruction has too many tokens");
            return LINE_ERROR;
        }
        uint16_t insn = 0;
        if ((nt == 0U) || !assemble_insn(ps->ctx, tok, nt, &insn)) {
            if (nt == 0U) {
                pa_set_error(ps->ctx, "empty instruction");
            }
            return LINE_ERROR;
        }
        ps->ctx->out->insns[ps->idx] = insn;
    }
    ps->idx++;
    return LINE_OK;
}

/* Process one cleaned, non-empty line. */
static line_result_t process_line(pa_parse_state_t *ps, char *line)
{
    if (ps->in_verbatim) {
        if ((line[0] == '%') && strchr(line, '}') != NULL) {
            ps->in_verbatim = false;
        }
        return LINE_OK;
    }
    if (line[0] == '%') {
        ps->in_verbatim = true;
        return LINE_OK;
    }
    if (line[0] == '.') {
        return handle_directive(ps, line);
    }
    if (!ps->in_program) {
        return LINE_OK;
    }
    if (handle_label(ps, &line)) {
        return LINE_OK; /* label-only line */
    }
    return handle_instruction(ps, line);
}

/* Run one pass over the source. Returns false (with error set) on failure. */
static bool run_pass(pa_parse_state_t *ps, const char *src)
{
    ps->in_program = false;
    ps->done_selected = false;
    ps->in_verbatim = false;
    ps->in_block_comment = false;
    ps->seen_any_program = false;
    ps->idx = 0;

    char buf[MAX_LINE];
    const char *cursor = src;
    int line_no = 0;

    while ((*cursor != '\0') && !ps->done_selected) {
        size_t k = 0;
        while ((*cursor != '\0') && (*cursor != '\n') && (k < (MAX_LINE - 1U))) {
            buf[k] = *cursor;
            cursor++;
            k++;
        }
        buf[k] = '\0';
        if (*cursor == '\n') {
            cursor++;
        }
        line_no++;
        ps->ctx->line_no = line_no;

        if (!ps->in_verbatim) {
            strip_block_comments(buf, &ps->in_block_comment);
        }
        char *line = clean_line(buf);
        if (line[0] == '\0') {
            continue;
        }
        line_result_t r = process_line(ps, line);
        if (r == LINE_ERROR) {
            return false;
        }
        if (r == LINE_DONE) {
            ps->done_selected = true;
        }
    }
    return true;
}

bool pio_asm_assemble(const char *src, const char *name, pio_program_t *out)
{
    (void)memset(out, 0, sizeof(*out));
    out->ok = true;
    out->wrap_bottom = 0;
    out->wrap_top = 0xFF;  /* sentinel: .wrap not yet seen */
    out->pio_version = -1; /* unset */

    asm_ctx_t ctx;
    (void)memset(&ctx, 0, sizeof(ctx));
    ctx.out = out;

    pa_parse_state_t ps = {&ctx, name, 1, false, false, false, false, false, 0};

    /* Pass 1: select the program, collect labels and side-set/wrap. */
    if (!run_pass(&ps, src)) {
        return false;
    }
    out->count = ps.idx;
    if (out->wrap_top == 0xFFU) {
        out->wrap_top = (ps.idx > 0U) ? (uint8_t)(ps.idx - 1U) : 0U;
    }
    if (ps.idx == 0U) {
        pa_set_error(&ctx, "no instructions found (check program name)");
        return false;
    }

    /* Pass 2: emit instruction words now that labels are known. */
    ps.pass = 2;
    if (!run_pass(&ps, src)) {
        return false;
    }

    /* Expose public-label offsets (mirrors pioasm's <prog>_offset_<name>). */
    for (uint8_t i = 0; i < ctx.label_count; i++) {
        if (ctx.labels[i].is_public && (out->public_count < PIO_ASM_MAX_PUBLIC)) {
            pio_public_label_t *pl = &out->public_labels[out->public_count];
            (void)snprintf(pl->name, sizeof(pl->name), "%s", ctx.labels[i].name);
            pl->index = ctx.labels[i].index;
            out->public_count++;
        }
    }
    return out->ok;
}

bool pio_asm_load_program(pio_sim_t *pio, uint8_t sm, uint8_t offset, const pio_program_t *prog)
{
    if (!prog->ok) {
        return false;
    }
    /* Relocate JMP targets by the load offset, as pio_add_program does for
     * position-independent programs (label indices were emitted relative to
     * program start). */
    uint16_t relocated[PIO_ASM_MAX_INSNS] = {0};
    for (uint8_t i = 0; i < prog->count; i++) {
        uint16_t insn = prog->insns[i];
        if (((insn >> 13U) == 0U /* JMP */) && (offset != 0U)) {
            uint8_t addr = (uint8_t)((insn & 0x1FU) + offset);
            insn = (uint16_t)((insn & ~0x1FU) | (addr & 0x1FU));
        }
        relocated[i] = insn;
    }
    pio_sim_load(pio, offset, relocated, prog->count);
    pio_sim_sm_set_wrap(pio, sm, (uint8_t)(offset + prog->wrap_bottom),
                        (uint8_t)(offset + prog->wrap_top));
    pio_sim_sm_set_sideset(pio, sm, prog->sideset_bits, prog->sideset_opt, prog->sideset_pindirs);
    /* Start at the program's first instruction (the SDK's pio_sm_init
     * convention), so any preamble before .wrap_target runs once. */
    pio_sim_sm_set_pc(pio, sm, offset);
    return true;
}

bool pio_asm_load_program_at_origin(pio_sim_t *pio, uint8_t sm, const pio_program_t *prog)
{
    uint8_t offset = prog->has_origin ? prog->origin : 0U;
    return pio_asm_load_program(pio, sm, offset, prog);
}

void pio_asm_apply_program_config(pio_sim_t *pio, uint8_t sm, const pio_program_t *prog)
{
    if (prog->has_clock_div) {
        /* 16.8 fixed-point, matching pico-sdk sm_config_set_clkdiv. */
        double d = prog->clock_div;
        if (d < 1.0) {
            d = 1.0;
        }
        uint32_t whole = (uint32_t)d;
        uint32_t frac = (uint32_t)(((d - (double)whole) * 256.0) + 0.5);
        if (frac > 255U) {
            frac = 0;
            whole++;
        }
        pio_sim_sm_set_clkdiv(pio, sm, (uint16_t)whole, (uint8_t)frac);
    }
    if (prog->has_fifo_join) {
        pio_sim_sm_set_fifo_join(pio, sm, prog->fifo_join);
    }
    if (prog->has_mov_status) {
        pio_sim_sm_set_status_sel(pio, sm, prog->mov_status_sel, prog->mov_status_n);
    }
    if (prog->in_cfg.set) {
        pio_sim_sm_set_in_shift(pio, sm, prog->in_cfg.dir, prog->in_cfg.autoshift,
                                prog->in_cfg.threshold);
#if PIO_SIM_HAS_IN_PIN_COUNT
        /* RP2350 only; RP2040 has no IN pin count, so the .in count is ignored there. */
        pio_sim_sm_set_in_pin_count(pio, sm, prog->in_cfg.count);
#endif
    }
    if (prog->out_cfg.set) {
        pio_sim_sm_set_out_shift(pio, sm, prog->out_cfg.dir, prog->out_cfg.autoshift,
                                 prog->out_cfg.threshold);
        pio_sim_sm_set_out_pin_count(pio, sm, prog->out_cfg.count);
    }
    if (prog->set_cfg.set) {
        pio_sim_sm_set_set_pin_count(pio, sm, prog->set_cfg.count);
    }
}
