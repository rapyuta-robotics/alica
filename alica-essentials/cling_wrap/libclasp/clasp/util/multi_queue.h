// 
// Copyright (c) 2010-2012, Benjamin Kaufmann
// 
// This file is part of Clasp. See http://www.cs.uni-potsdam.de/clasp/ 
// 
// Clasp is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
// 
// Clasp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with Clasp; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
#ifndef CLASP_MULIT_QUEUE_H_INCLUDED
#define CLASP_MULIT_QUEUE_H_INCLUDED

#include <clasp/util/platform.h>
#include <clasp/util/atomic.h>

namespace Clasp { namespace mt { namespace Detail {
struct NodeBase {
	typedef Clasp::atomic<NodeBase*> AtomicPtr;
	typedef Clasp::atomic<int>       AtomicInt;
	explicit NodeBase(uint32 rc) { next = 0; refs = rc; }
	AtomicPtr next;
	AtomicInt refs;
};

template <class T>
struct Node : public NodeBase {
	Node(uint32 rc, const T& d) : NodeBase(rc), data(d) {}
	T data;
};
struct DefaultDeleter { 
	template <class T> 
	void operator()(T& obj) const {
		(void)obj;
		obj.~T();
	} 
};
}

//! A (base) class for distributing items between n different threads
/*!
 * Logically, the class maintains n queues, one for each
 * involved thread. Threads must register themselves by
 * calling addThread(). The returned handle has then
 * to be used for publishing and consuming items.
 */
template <class T, class Deleter = Detail::DefaultDeleter>
class MultiQueue {
protected:
	typedef Detail::Node<T>   Node;
	typedef Detail::NodeBase  NodeBase;
public:
	typedef Detail::NodeBase* ThreadId;
	//! creates a new object for at most m threads
	explicit MultiQueue(uint32 m, const Deleter& d = Deleter()) : head_(m+1), maxQ_(m), deleter_(d) {
		tail_         = &head_;
	}
	uint32 maxThreads() const { return maxQ_; }
	void reserve(uint32 c) {
		for (uint32 i = 0; i != c; ++i) {
			void* m = ::operator new(sizeof(Node));
			freeList_.push(new (m) NodeBase(0));
		}
	}
	//! destroys the object and all unconsumed items
	~MultiQueue() {
		for (NodeBase* x = head_.next; x ; ) {
			Node* n = static_cast<Node*>(x);
			x = x->next;
			deleter_(n->data);
			freeList_.push(n);
		}
	}
	//! adds a new thread to the object 
	/*!
	 * \note Shall be called at most m times
	 * \return A handle identifying the new thread
	 */
	ThreadId addThread() {
		--head_.refs;
		assert(head_.refs > 0);
		return &head_;
	}
	bool hasItems(ThreadId& cId) const { return cId != tail_; }
	
	//! tries to consume an item
	/*!
	 * \pre cId was initially obtained via a call to addThread()
	 * \note tryConsume() is thread-safe w.r.t different ThreadIds
	 */
	bool tryConsume(ThreadId& cId, T& out) {
		if (cId != tail_) {
			NodeBase* n = cId;
			cId         = cId->next;
			assert(cId != 0 && "MultiQueue is corrupted!");
			release(n);
			out = static_cast<Node*>(cId)->data;
			return true;
		}
		return false;
	}
	//! pops an item from the queue associated with the given thread
	/*! 
	 * \pre hasItems(cId) == true
	 */
	void pop(ThreadId& cId) {
		assert(hasItems(cId) && "Cannot pop from empty queue!");
		NodeBase* n = cId;
		cId         = cId->next;
		release(n);
	}
protected:
	//! publishes a new item
	/*!
	 * \note the function is *not* thread-safe, i.e.
	 * it must not be called concurrently
	 */
	void unsafePublish(const T& in, const ThreadId&) {
		Node* n     = freeList_.allocate(in, maxQ_);
		publishRelaxed(n);
	}

	//! concurrency-safe version of unsafePublish
	void publish(const T& in, const ThreadId&) {
		Node* newNode = freeList_.allocate(in, maxQ_);
		NodeBase* assumedTail, *assumedNext;
		do {
			assumedTail = tail_;
			assumedNext = assumedTail->next;
			if (assumedTail != tail_) { 
				// tail has changed - try again
				continue; 
			}
			if (assumedNext != 0) {
				// someone has added a new node but has not yet
				// moved the tail - assist him and start over
				tail_.compare_and_swap(assumedNext, assumedTail); 
				continue;
			}
		} while (assumedTail->next.compare_and_swap(newNode, 0) != 0);
		// Now that we managed to link a new node to what we think is the current tail
		// we try to update the tail. If the tail is still what we think it is, 
		// it is moved - otherwise some other thread already did that for us.
		tail_.compare_and_swap(newNode, assumedTail);
	}

	//! Non-atomically adds n to the global queue
	void publishRelaxed(NodeBase* n) {
		tail_->next = n;
		tail_       = n;
	}
	uint32 maxQ() const { return maxQ_; }
	Node*  allocate(uint32 maxR, const T& in) {
		return freeList_.allocate(in, maxR);
	}
private:
	MultiQueue(const MultiQueue&);
	MultiQueue& operator=(const MultiQueue&);
	// Stack of free nodes
	struct FreeList {
		FreeList() { top = 0; }
		~FreeList(){
			for (NodeBase* n = top; n != 0; ) {
				NodeBase* t = n;
				n = n->next;
				::operator delete(t);
			}
		}
		void push(NodeBase* n) {
			NodeBase* assumedTop;
			do {
				assumedTop = top;
				n->next    = assumedTop;
			} while (top.compare_and_swap(n, assumedTop) != assumedTop);
		}
		NodeBase* tryPop() {
			NodeBase* n = 0, *next = 0;
			do {
				n = top;
				if (!n) return 0;
				// NOTE: 
				// If the queue is used correctly, n is
				// safe and n->next is ABA-safe at this point.
				// The ref-counting in the queue makes sure
				// that a node (here n) cannot be added back
				// to the free list while another thread 
				// is still in tryPop() - that thread had
				// not yet the chance to decrease the node's
				// ref count.
				next = n->next;
			} while (top.compare_and_swap(next, n) != n);
			return n;
		}
		Node* allocate(const T& in, uint32 maxRef) {
			if (NodeBase* n = tryPop()) {
				return new (n) Node(maxRef, in);
			}
			else { 
				void* mem = ::operator new(sizeof(Node));
				return new (mem) Node(maxRef, in);
			}
		}
		NodeBase::AtomicPtr top;
	};
	
	void release(NodeBase* n) {
		if (n != &head_ && --n->refs == 0) {
			head_.next = n->next;
			deleter_(static_cast<Node*>(n)->data);
			freeList_.push(n);
		}
	}
	NodeBase            head_;
	NodeBase::AtomicPtr tail_;
	FreeList            freeList_;
	const uint32        maxQ_;
	Deleter             deleter_;
};

} } // end namespace Clasp::mt
#endif
