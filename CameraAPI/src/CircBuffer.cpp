#include "CircBuffer.h"

/*
// Thread safe circular buffer 


template<typename T>
inline CircBuffer<T>::CircBuffer()
{
}

template<typename T>
inline CircBuffer<T>::CircBuffer(int n)
{
	cb.set_capacity(n);
}

template<typename T>
inline CircBuffer<T>::~CircBuffer()
{
}

template<typename T>
inline void CircBuffer<T>::send(T imdata)
{
	lock lk(monitor);
	cb.push_back(imdata);
	buffer_not_empty.notify_one();
}

template<typename T>
inline T CircBuffer<T>::receive()
{
	lock lk(monitor);
	while (cb.empty())
		buffer_not_empty.wait(lk);
	T imdata = cb.front();
	cb.pop_front();
	return imdata;
}

template<typename T>
inline void CircBuffer<T>::clear()
{
	lock lk(monitor);
	cb.clear();
}

template<typename T>
inline int CircBuffer<T>::size()
{
	lock lk(monitor);
	return cb.size();
}

template<typename T>
inline void CircBuffer<T>::set_capacity(int capacity)
{
	lock lk(monitor);
	cb.set_capacity(capacity);
}

*/

