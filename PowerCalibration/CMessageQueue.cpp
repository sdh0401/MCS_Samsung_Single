#include "pch.h"
#include "CMessageQueue.h"
#include "GlobalDefine.h"
#include "Trace.h"
#include "GlobalData.h"
#include "LockDef.h"

#include <iostream>
#include <queue>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <ctime>
#include <string>

using namespace std;

CMessageQueue::CMessageQueue(int nMaxCount)
{
	CNode* pFirstNode = new CNode;
	// The queue
	// QHead = QTail = node # Both Head and Tail point to it
	pHeadNode = pFirstNode;
	pTailNode = pFirstNode;
	// Queue size, dummy node counted off
	m_Size = 0;
	m_nMaxSize = nMaxCount;
	m_EventReceive = CreateSemaphore(NULL, 0, nMaxCount, NULL);
}

CMessageQueue::~CMessageQueue()
{
	void* pDummyValue = NULL;
	while (Pop(pDummyValue));
	if (pHeadNode) { try { delete pHeadNode; } catch (...) { NULL; } }
	CloseHandle(m_EventReceive);
}

bool CMessageQueue::Push(void* pNewValue)
{
    // node = new node() # Allocate a new node from the free list
    // node->next = NULL # Set next pointer of node to NULL
    CNode* pNewNode = new CNode;
    // node->value = value # Copy enqueued value into node
    pNewNode->SetValue(pNewValue);
    while (m_Size > m_nMaxSize) { Sleep(100); }
    // lock(&QTlock) # Acquire T lock in order to access Tail
    CSingleLock singleLock(&m_TailCriticalSection);
    singleLock.Lock();
    // QTail->next = node # Link node at the end of the linked list
    pTailNode->SetNext(pNewNode);
    // QTail = node # Swing Tail to node
    pTailNode = pNewNode;
    // Increment size - use InterlockedIncrement for accurate sizes
    // ::InterlockedIncrement(&m_Size);
    m_Size++;
    // unlock(&QTlock) # Release T lock
    singleLock.Unlock();
    while (ReleaseSemaphore(m_EventReceive, 1, NULL) == FALSE) Sleep(100);
    return true;
}

bool CMessageQueue::Pop(void*& pValue, int nMiliSeconds)
{
    // lock(&QH lock) # Acquire H lock in order to access Head
 //	int nSize;
    //DWORD dwWaitResult;
    //	singleLock.Lock();
    //	nSize = m_Size;
    //	singleLock.Unlock();
    if (WaitForSingleObject(m_EventReceive, nMiliSeconds) != WAIT_OBJECT_0) { return false; }
    CSingleLock singleLock(&m_HeadCriticalSection);
    singleLock.Lock();
    // node = Q->Head # Read Head
    CNode* pCurrentNode = pHeadNode;
    // new_head = node->next # Read next pointer
    CNode* pNewHeadNode = pHeadNode->GetNext();
    // if new_head == NULL # Is queue empty?
    if (NULL == pNewHeadNode) // # Queue was empty
    {
        //    unlock(&QH lock) # Release H lock before return
        m_Size = 0;
        singleLock.Unlock();
        //    return FALSE
        return false;
    }
    // endif
    // *pvalue = new_head->value # Queue not empty. Read value before release
    pValue = pNewHeadNode->GetValue();
    // QHead = new_head # Swing Head to next node
    pHeadNode = pNewHeadNode;
    // decrement size - use InterlockedDecrement for accurate sizes
    // ::InterlockedDecrement(&m_Size);
    m_Size--;
    // unlock(&QH lock) # Release H lock
    singleLock.Unlock();
    // free(node) # Free node
    try { delete pCurrentNode; }
    catch (...) { NULL; }
    // return TRUE # Queue was not empty, dequeue succeeded
    return true;
}

void CMessageQueue::Flush()
{
    void* pVoid;
    CSingleLock singleLock(&m_TailCriticalSection);
    singleLock.Lock();
    while (TRUE)
    {
        if (Pop(pVoid) == false) break;
        delete pVoid;
    }
    singleLock.Unlock();
}

