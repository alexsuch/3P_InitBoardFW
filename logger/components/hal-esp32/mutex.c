#include <hal/mutex.h>

void mutex_open(mutex_t *mutex) {
    mutex->sema = xSemaphoreCreateMutex();
    assert(mutex->sema);
    // Mutexes are created in an unlocked state, no need to unlock
}

void mutex_lock(mutex_t *mutex) { xSemaphoreTake(mutex->sema, portMAX_DELAY); }

void mutex_unlock(mutex_t *mutex) { xSemaphoreGive(mutex->sema); }

void mutex_close(mutex_t *mutex) {
    vSemaphoreDelete(mutex->sema);
    mutex->sema = NULL;
}