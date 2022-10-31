#ifndef AUTOPTR_H
#define AUTOPTR_H
#include <assert.h>

namespace GMapping{

template <class X>
class autoptr{
	protected:
	
	public:
	struct reference{
		X* data;
		unsigned int shares;
	};
		inline autoptr(X* p=(X*)(0));
		inline autoptr(const autoptr<X>& ap);
		inline autoptr& operator=(const autoptr<X>& ap);
		inline ~autoptr();
		inline operator int() const;
		inline X& operator*();
		inline const X& operator*() const;
		//p	
		reference * m_reference;
	protected:
};

template <class X>
autoptr<X>::autoptr(X* p){
	m_reference=0;
	if (p){
		m_reference=new reference;
		m_reference->data=p;
		m_reference->shares=1;
	}
}

template <class X>
autoptr<X>::autoptr(const autoptr<X>& ap){
	m_reference=0;
	reference* ref=ap.m_reference;
	if (ap.m_reference){
		m_reference=ref;
		m_reference->shares++;
	}
}

template <class X>
autoptr<X>& autoptr<X>::operator=(const autoptr<X>& ap){
	reference* ref=ap.m_reference;
	if (m_reference==ref){
		return *this;
	}
	if (m_reference && !(--m_reference->shares)){
		delete m_reference->data;
		delete m_reference;
		m_reference=0;
	}	
	if (ref){
		m_reference=ref;
		m_reference->shares++;
	} 
//20050802 nasty changes begin
	else
		m_reference=0;
//20050802 nasty changes end
	return *this;
}

template <class X>
autoptr<X>::~autoptr(){
	if (m_reference && !(--m_reference->shares)){
		delete m_reference->data;
		delete m_reference;
		m_reference=0;
	}	
}

template <class X>
autoptr<X>::operator int() const{
	return m_reference && m_reference->shares && m_reference->data;
}

template <class X>
X& autoptr<X>::operator*(){
	assert(m_reference && m_reference->shares && m_reference->data);
	return *(m_reference->data);
}

template <class X>
const X& autoptr<X>::operator*() const{
	assert(m_reference && m_reference->shares && m_reference->data);
	return *(m_reference->data);
}

};
#endif
