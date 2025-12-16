#pragma once

#include <queue>
#include <string>
using namespace std;
class CNode
{
public:
    CNode() noexcept = default;
    ~CNode() = default;

    CNode(const CNode&) = delete;
    CNode& operator=(const CNode&) = delete;

    void SetValue(void* p) { pValuePointer = p; }
    void* GetValue() const { return pValuePointer; }

    void SetNext(CNode * p) { pNextNode = p; }
    CNode* GetNext() const { return pNextNode; }

private:
    CNode* pNextNode = nullptr;
    void* pValuePointer = nullptr;
};

class CMessageQueue
{
public:
    CMessageQueue(int nMaxCount = 500);
    virtual ~CMessageQueue(void);

    // Don't allow these sorts of things
    CMessageQueue(const CMessageQueue&) = delete;
    CMessageQueue& operator=(const CMessageQueue&) = delete;

private:
    HANDLE m_EventReceive;
protected:
    // Critical sections guarding Head and Tail code sections
    CCriticalSection m_HeadCriticalSection;
    CCriticalSection m_TailCriticalSection;
    // The queue, two pointers to head and tail respectively
    CNode* pHeadNode;
    CNode* pTailNode;
    // Queue size
    volatile int m_Size;
    int m_nMaxSize;

public:
    // Enqueue
    //void SetAsEventType();
    bool Push(void* pNewValue);
    // Dequeue, pass a pointer by reference
    bool Pop(void*& pValue, int nMiliseconds = 0);
    // for accurate sizes change the code to use the Interlocked functions calls
    int GetSize() { return m_Size; }
    void Flush();
};
