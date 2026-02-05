#pragma once

typedef struct _List List;

struct _List {
    void **pListPointArray;                         // LIST array pointer
    int Total;                                      // Number of elements
    void (*Add)(List *pList, void *pListPoint);     // Add
    void (*Remove)(List *pList, void *pListPoint);  // Remove
    void (*Delete)(void *pList);                    // Destructor
};

List *ListCreate(void);