#include "list.h"

#include "freertos/FreeRTOS.h"
#include "stddef.h"

// Destructor function for the List class
static void ListDelete(void *pList) {
    // First release the pointer array
    if (((List *)pList)->pListPointArray != NULL) {
        free(((List *)pList)->pListPointArray);
    }
    // Then release the entire List class
    free(pList);
}
// Function to add an element
static void ListAdd(List *pList, void *pListPoint) {
    // Allocate memory that is one unit larger than before
    void **tListPointArray = malloc(sizeof(int *) * (pList->Total + 1));
    int pListIndex;
    // Copy
    for (pListIndex = 0; pListIndex < pList->Total; pListIndex++) {
        // Check if the same element already exists
        if (pList->pListPointArray[pListIndex] == pListPoint) {
            // Release the newly allocated memory
            free(tListPointArray);
            // Return
            return;
        }
        // Copy
        tListPointArray[pListIndex] = pList->pListPointArray[pListIndex];
    }

    tListPointArray[pList->Total] = pListPoint;
    // Increment the total count by 1
    pList->Total += 1;
    // Release the original memory
    if (pList->pListPointArray != NULL) free(pList->pListPointArray);
    // Replace the original handle with the new one
    pList->pListPointArray = tListPointArray;
}
// Function to remove an element
static void ListRemove(List *pList, void *pListPoint) {
    int pListIndex, tListIndex;
    void **tListPointArray;
    void **FreePointArray;
    void **SavePointArray;

    // Exit if the total count is 0
    if (pList->Total == 0) return;

    // Allocate memory that is one unit smaller than before
    tListPointArray = malloc(sizeof(int) * (pList->Total - 1));
    // Use the newly allocated memory as the default space to be freed
    FreePointArray = tListPointArray;
    // Use the existing memory as the default storage space
    SavePointArray = pList->pListPointArray;

    // Find the point to remove
    for (pListIndex = 0, tListIndex = 0; pListIndex < pList->Total; pListIndex++) {
        // The current point is the one to remove
        if (pList->pListPointArray[pListIndex] == pListPoint) {
            // Change the pointer to the memory to be freed
            FreePointArray = pList->pListPointArray;
            // Change the pointer to the memory to be kept
            SavePointArray = tListPointArray;
            // End this iteration
            continue;
        }

        // If the current point is not the one to remove, and the copy index is less than total-1
        if (tListIndex < (pList->Total - 1)) {
            // Copy
            tListPointArray[tListIndex] = pList->pListPointArray[pListIndex];
            // Increment the copy index by 1
            tListIndex++;
        }
    }

    // Change the total count based on the kept memory block
    pList->Total = (SavePointArray == tListPointArray) ? pList->Total - 1 : pList->Total;
    // Release the unused memory block that should be freed
    if (FreePointArray != NULL) free(FreePointArray);
    // Keep the memory block that should be kept
    pList->pListPointArray = SavePointArray;
}

// Constructor function for List
List *ListCreate(void) {
    List *pList = (List *)malloc(sizeof(List));
    pList->Total = 0;
    pList->pListPointArray = NULL;
    pList->Add = ListAdd;
    pList->Remove = ListRemove;
    pList->Delete = ListDelete;
    return pList;
}