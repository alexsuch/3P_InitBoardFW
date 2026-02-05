#include "util/event_sender.h"

#define ES_MAX_EMIT_DEPTH 8

static inline void es_lock(es_t *es) {
    if (es->mtx && es->mtx->lock) {
        es->mtx->lock(es->mtx->ctx);
    }
}

static inline void es_unlock(es_t *es) {
    if (es->mtx && es->mtx->unlock) {
        es->mtx->unlock(es->mtx->ctx);
    }
}

static void es_free_slot(es_t *es, uint16_t idx) {
    es_slot_t *slot = &es->slots[idx];
    slot->alive = 0;
    es->free_stack[es->free_top++] = idx;
}

void es_init(es_t *es, es_slot_t *slots, uint16_t capacity, uint16_t *free_stack, es_mutex_iface_t *mtx) {
    es->slots = slots;
    es->capacity = capacity;
    es->free_stack = free_stack;
    es->mtx = mtx;
    es->emit_depth = 0;
    es->free_top = capacity;
    for (uint16_t i = 0; i < capacity; ++i) {
        slots[i].handler = NULL;
        slots[i].user = NULL;
        slots[i].flags = 0;
        slots[i].generation = 0;
        slots[i].alive = 0;
        free_stack[i] = capacity - 1 - i;
    }
}

int es_subscribe(es_t *es, es_handler_t handler, void *user, uint16_t flags, es_token_t *out) {
    if (!handler) {
        return -1;
    }
    es_lock(es);
    if (es->free_top == 0) {
        es_unlock(es);
        return -1;
    }
    uint16_t idx = es->free_stack[--es->free_top];
    es_slot_t *slot = &es->slots[idx];
    slot->handler = handler;
    slot->user = user;
    slot->flags = flags;
    slot->generation++;
    slot->alive = 1;
    es_unlock(es);
    if (out) {
        out->idx = idx;
        out->gen = slot->generation;
    }
    return 0;
}

void es_unsubscribe(es_t *es, es_token_t tok) {
    if (!es || tok.idx >= es->capacity) {
        return;
    }
    es_lock(es);
    es_slot_t *slot = &es->slots[tok.idx];
    if (slot->generation != tok.gen || !slot->alive) {
        es_unlock(es);
        return;
    }
    es_free_slot(es, tok.idx);
    es_unlock(es);
}

void es_emit(es_t *es, const es_event_t *ev) {
    if (!ev || es->emit_depth >= ES_MAX_EMIT_DEPTH) {
        return;
    }
    es_slot_t *snapshot[es->capacity];
    uint16_t gen_snapshot[es->capacity];
    uint16_t count = 0;

    es_lock(es);
    for (uint16_t i = 0; i < es->capacity; ++i) {
        if (es->slots[i].alive) {
            snapshot[count] = &es->slots[i];
            gen_snapshot[count] = es->slots[i].generation;
            ++count;
        }
    }
    es_unlock(es);

    es->emit_depth++;
    for (uint16_t i = 0; i < count; ++i) {
        es_slot_t *slot = snapshot[i];
        uint16_t gen = gen_snapshot[i];
        if (!slot->alive || slot->generation != gen) {
            continue;
        }
        slot->handler(ev, slot->user);
        if ((slot->flags & ESF_ONESHOT) && slot->alive && slot->generation == gen) {
            es_token_t tok = {(uint16_t)(slot - es->slots), gen};
            es_unsubscribe(es, tok);
        }
    }
    es->emit_depth--;
}

void es_emit_from_isr(es_t *es, const es_event_t *ev) {
    if (!ev || es->emit_depth >= ES_MAX_EMIT_DEPTH) {
        return;
    }
    es_event_t isr_ev = *ev;
    isr_ev.meta_flags |= ESF_FROM_ISR;
    es_slot_t *snapshot[es->capacity];
    uint16_t gen_snapshot[es->capacity];
    uint16_t count = 0;
    for (uint16_t i = 0; i < es->capacity; ++i) {
        if (es->slots[i].alive) {
            snapshot[count] = &es->slots[i];
            gen_snapshot[count] = es->slots[i].generation;
            ++count;
        }
    }
    es->emit_depth++;
    for (uint16_t i = 0; i < count; ++i) {
        es_slot_t *slot = snapshot[i];
        uint16_t gen = gen_snapshot[i];
        if (!slot->alive || slot->generation != gen) {
            continue;
        }
        slot->handler(&isr_ev, slot->user);
        if ((slot->flags & ESF_ONESHOT) && slot->alive && slot->generation == gen) {
            es_lock(es);
            es_free_slot(es, (uint16_t)(slot - es->slots));
            es_unlock(es);
        }
    }
    es->emit_depth--;
}
