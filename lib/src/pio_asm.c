/*
 * SPDX-License-Identifier: MIT
 * pio_asm — minimal PIO assembler for pio_sim. See pio_asm.h.
 */

#include "pio_asm.h"

#include <stdio.h>
#include <string.h>

#define MAX_LABELS 32U
#define MAX_LINE 256U
#define MAX_TOKENS 12U

typedef struct {
    char name[32];
    uint8_t index;
    bool is_public;
} label_t;

typedef struct {
    pio_program_t *out;
    label_t labels[MAX_LABELS];
    uint8_t label_count;
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
    }
    for (; *s; s++) {
        uint32_t digit;
        if ((*s >= '0') && (*s <= '9')) {
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

/* Tokenize `line` in place on whitespace and commas. Returns token count. */
static uint8_t tokenize(char *line, char *tok[], uint8_t max)
{
    uint8_t n = 0;
    char *p = line;
    while (*p && (n < max)) {
        while ((*p == ' ') || (*p == '\t') || (*p == ',')) {
            p++;
        }
        if (*p == '\0') {
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

/* MOV destination name → field. */
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
            if (!parse_uint(&last[1], &d)) {
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
            if (!parse_uint(tok[*n - 1U], &sv)) {
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
    const char *target = tok[1];
    if (n >= 3U) {
        const char *c = tok[1];
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
            pa_set_error(ctx, "unknown jmp condition");
            return false;
        }
        target = tok[2];
    }
    uint32_t addr;
    int lbl = find_label(ctx, target);
    if (lbl >= 0) {
        addr = (uint32_t)lbl;
    } else if (!parse_uint(target, &addr)) {
        pa_set_error(ctx, "unknown jmp target/label");
        return false;
    } else {
        /* Address already resolved from a label. */
    }
    *base = pio_sim_encode_jmp(cond, (uint8_t)addr);
    return true;
}

static bool enc_wait(asm_ctx_t *ctx, char *tok[], uint8_t n, uint16_t *base)
{
    if (n < 4U) {
        pa_set_error(ctx, "wait needs <pol> <src> <index>");
        return false;
    }
    uint32_t pol;
    if (!parse_uint(tok[1], &pol)) {
        pa_set_error(ctx, "bad wait polarity");
        return false;
    }
    uint8_t source;
    if (ieq(tok[2], "gpio")) {
        source = PIO_WAIT_GPIO;
    } else if (ieq(tok[2], "pin")) {
        source = PIO_WAIT_PIN;
    } else if (ieq(tok[2], "irq")) {
        source = PIO_WAIT_IRQ;
    } else {
        pa_set_error(ctx, "bad wait source");
        return false;
    }
    uint32_t idx;
    if (!parse_uint(tok[3], &idx)) {
        pa_set_error(ctx, "bad wait index");
        return false;
    }
    bool rel = (n > 4U) && ieq(tok[4], "rel");
    if (rel && (source != PIO_WAIT_IRQ)) {
        pa_set_error(ctx, "rel only valid for wait irq");
        return false;
    }
    *base = rel ? pio_sim_encode_wait_irq_rel((uint8_t)pol, (uint8_t)idx)
                : pio_sim_encode_wait((uint8_t)pol, source, (uint8_t)idx);
    return true;
}

static bool enc_in(asm_ctx_t *ctx, char *tok[], uint8_t n, uint16_t *base)
{
    if (n < 3U) {
        pa_set_error(ctx, "in needs <src>, <count>");
        return false;
    }
    int src = src_field(tok[1]);
    uint32_t cnt;
    if ((src < 0) || !parse_uint(tok[2], &cnt)) {
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
    if ((dest < 0) || !parse_uint(tok[2], &cnt)) {
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

static bool enc_mov(asm_ctx_t *ctx, char *tok[], uint8_t n, uint16_t *base)
{
    if (n < 3U) {
        pa_set_error(ctx, "mov needs <dest>, <src>");
        return false;
    }
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
    uint32_t idx = 0;
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
        } else if (parse_uint(tok[i], &idx)) {
            have_idx = true;
        } else {
            pa_set_error(ctx, "bad irq operand");
            return false;
        }
    }
    if (!have_idx) {
        pa_set_error(ctx, "irq needs an index");
        return false;
    }
    *base = rel ? pio_sim_encode_irq_rel(clear, wait, (uint8_t)idx)
                : pio_sim_encode_irq(clear, wait, (uint8_t)idx);
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
    if ((dest < 0) || !parse_uint(tok[2], &val)) {
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
static char *clean_line(char *line)
{
    char *semi = strchr(line, ';');
    if (semi != NULL) {
        *semi = '\0';
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
    bool seen_any_program;
    uint8_t idx; /* instruction index within the program */
} pa_parse_state_t;

typedef enum {
    LINE_OK,    /* processed, continue */
    LINE_DONE,  /* stop scanning (next program reached) */
    LINE_ERROR, /* error set on ctx */
} line_result_t;

/* Tokenize a copy of `line` (preserving the original). Returns token count. */
static uint8_t tokenize_copy(const char *line, char tmp[MAX_LINE], char *tok[MAX_TOKENS])
{
    int written = snprintf(tmp, MAX_LINE, "%s", line);
    (void)written;
    for (uint8_t i = 0; i < MAX_TOKENS; i++) {
        tok[i] = NULL;
    }
    return tokenize(tmp, tok, MAX_TOKENS);
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

static line_result_t handle_directive(pa_parse_state_t *ps, const char *line)
{
    char tmp[MAX_LINE];
    char *tok[MAX_TOKENS];
    uint8_t nt = tokenize_copy(line, tmp, tok);
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
        ps->idx = 0;
        return LINE_OK;
    }
    if (!ps->in_program) {
        return LINE_OK;
    }
    if (ieq(tok[0], ".side_set")) {
        if (ps->pass == 1) {
            uint32_t bits = 0;
            if ((nt < 2U) || !parse_uint(tok[1], &bits)) {
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
    } else {
        /* Not a wrap directive. */
    }
    /* .define / .origin / .lang_opt etc.: ignored */
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
        uint8_t nt = tokenize_copy(line, tmp, tok);
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
    out->wrap_top = 0xFF; /* sentinel: .wrap not yet seen */

    asm_ctx_t ctx;
    (void)memset(&ctx, 0, sizeof(ctx));
    ctx.out = out;

    pa_parse_state_t ps = {&ctx, name, 1, false, false, false, false, 0};

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
    pio_sim_sm_set_pc(pio, sm, (uint8_t)(offset + prog->wrap_bottom));
    return true;
}
