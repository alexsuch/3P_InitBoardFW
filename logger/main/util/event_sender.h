#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t id;          // Event identifier
    const void *payload;  // Pointer to payload data
    uint16_t size;        // Size of payload
    uint16_t meta_flags;  // Meta information flags
} es_event_t;

typedef void (*es_handler_t)(const es_event_t *ev, void *user);

typedef struct {
    uint16_t idx;
    uint16_t gen;
} es_token_t;

enum {
    ESF_FROM_ISR = 1 << 0,
    ESF_ONESHOT = 1 << 1,
};

typedef struct {
    // lock/unlock form a critical section protecting the free-slot stack.
    // They must be safe to call from both task and ISR context when
    // ESF_ONESHOT handlers can fire via es_emit_from_isr (e.g., disable
    // interrupts inside the callbacks).
    void (*lock)(void *ctx);
    void (*unlock)(void *ctx);
    void *ctx;
} es_mutex_iface_t;

typedef struct {
    es_handler_t handler;
    void *user;
    uint16_t flags;
    uint16_t generation;
    volatile uint8_t alive;
} es_slot_t;

typedef struct {
    es_slot_t *slots;
    uint16_t *free_stack;
    uint16_t capacity;
    uint16_t free_top;
    es_mutex_iface_t *mtx;
    uint16_t emit_depth;
} es_t;

void es_init(es_t *es, es_slot_t *slots, uint16_t capacity, uint16_t *free_stack, es_mutex_iface_t *mtx);
int es_subscribe(es_t *es, es_handler_t handler, void *user, uint16_t flags, es_token_t *out);
void es_unsubscribe(es_t *es, es_token_t token);
void es_emit(es_t *es, const es_event_t *ev);
void es_emit_from_isr(es_t *es, const es_event_t *ev);

#ifdef __cplusplus
}
#endif
