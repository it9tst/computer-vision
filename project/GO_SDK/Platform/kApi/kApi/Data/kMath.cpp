/** 
 * @file    kMath.cpp
 *
 * @internal
 * Copyright (C) 2005-2021 by LMI Technologies Inc.
 * Licensed under the MIT License.
 * Redistributed files must retain the above copyright notice.
 */
#include <kApi/Data/kMath.h>
#include <math.h>

kBeginStaticClassEx(k, kMath)
kEndStaticClassEx()

kFx(kStatus) xkMath_InitStatic()
{
    return kOK;
}

kFx(kStatus) xkMath_ReleaseStatic()
{
    return kOK;
}

template<typename T> 
struct kIsEq
{
    bool operator()(const T& v1, const T& v2) const { return v1 == v2; }
};

template<typename T> 
struct kIsNeq
{
    bool operator()(const T& v1, const T& v2) const { return v1 != v2; }
};

template<typename T> 
struct kIsLt
{
    bool operator()(const T& v1, const T& v2) const { return v1 < v2; }
};

template<typename T> 
struct kIsLte
{
    bool operator()(const T& v1, const T& v2) const { return v1 <= v2; }
};

template<typename T> 
struct kIsGt
{
    bool operator()(const T& v1, const T& v2) const { return v1 > v2; }
};

template<typename T> 
struct kIsGte
{
    bool operator()(const T& v1, const T& v2) const { return v1 >= v2; }
};

template<typename T>
struct kAdd
{
    T operator()(const T& v1, const T& v2) const { return v1 + v2; }
};

template<typename T>
struct kSubtract
{
    T operator()(const T& v1, const T& v2) const { return v1 - v2; }
};

template<typename T>
struct kMultiply
{
    T operator()(const T& v1, const T& v2) const { return v1 * v2; }
};

template<typename T>
struct kDivide
{
    T operator()(const T& v1, const T& v2) const { return v1 / v2; }
};

template<typename T, typename P> 
kStatus kMathFindFirst(const T* v, kSize count, const P& pred, const T& value, kSize* index)    
{                                                                                           
    auto it = v;                                                                     
    auto end = it + count;                                                           
                                                                                            
    if ((!v && (count > 0)) || !index)                                                      
    {                                                                                       
        return kERROR_PARAMETER;                                                            
    }                                                                                       
                                                                                            
    *index = count;                                                                         
                                                                                            
    if (count > 0)                                                                          
    {                                                                                       
        while (it != end)                                                                   
        {                                                                                   
            if (pred(*it, value))                                                 
            {                                                                               
                *index = (kSize) (it - v);                                                  
                break;                                                                      
            }                                                                               
            it++;                                                                           
        }                                                                                   
    }                                                                                       
                                                                                            
    return kOK;                                                                             
}

template<typename T, typename P> 
kStatus kMathFindLast(const T* v, kSize count, const P& pred, const T& value, kSize* index)
{                                                                                           
    const T* it = v + count - 1;                                                         
    const T* end = it - count;                                                           
                                                                                            
    if ((!v && (count > 0)) || !index)                                                      
    {                                                                                       
        return kERROR_PARAMETER;                                                            
    }                                                                                       
                                                                                            
    *index = count;                                                                         
                                                                                            
    if (count > 0)                                                                          
    {                                                                                       
        while (it != end)                                                                   
        {                                                                                   
            if (pred(*it, value))                                                 
            {                                                                               
                *index = (kSize) (it - v);                                                  
                break;                                                                      
            }                                                                               
            it--;                                                                           
        }                                                                                   
    }                                                                                       
                                                                                            
    return kOK;                                                                             
}

template<typename T> 
kStatus kMathFindMin(const T* v, kSize count, kSize* index)
{                                                                                           
    const T* it = v;                                                                     
    const T* end = it + count;                                                           
    const T* minIt = it;                                                                 
                                                                                            
    if (!it || (count == 0) || !index)                                                      
    {                                                                                       
        return kERROR_PARAMETER;                                                            
    }                                                                                       
                                                                                            
    while (it != end)                                                                       
    {                                                                                       
        if (*it < *minIt)                                                                   
        {                                                                                   
            minIt = it;                                                                     
        }                                                                                   
        it++;                                                                               
    }                                                                                       
                                                                                            
    *index = (kSize) (minIt - v);                                                           
    return kOK;                                                                             
}

template<typename T> 
kStatus kMathFindMax(const T* v, kSize count, kSize* index)
{                                                                                           
    const T* it = v;                                                                     
    const T* end = it + count;                                                           
    const T* maxIt = it;                                                                 
                                                                                            
    if (!it || (count == 0) || !index)                                                      
    {                                                                                       
        return kERROR_PARAMETER;                                                            
    }                                                                                       
                                                                                            
    while (it != end)                                                                       
    {                                                                                       
        if (*it > *maxIt)                                                                   
        {                                                                                   
            maxIt = it;                                                                     
        }                                                                                   
        it++;                                                                               
    }                                                                                       
                                                                                            
    *index = (kSize) (maxIt - v);                                                           
    return kOK;                                                                             
}

template<typename T, typename V> 
kStatus kMathSum(const T* v, kSize count, V* sum)
{                                                                                          
    const T* it = v;                                                                    
    const T* itEnd = it + count;                                                        
    V accum = 0;                                                                        
                                                                                           
    if ((!v && (count > 0))  || !sum)                                                      
    {                                                                                      
        return kERROR_PARAMETER;                                                           
    }                                                                                      
                                                                                           
    while (it != itEnd)                                                                    
    {                                                                                      
        accum += (V) (*it++);                                                           
    }                                                                                      
                                                                                           
    *sum = accum;                                                                          

    return kOK;                                                                            
}

template<typename T> 
kStatus kMathAverage(const T* v, kSize count, k64f* average)
{                                                                                           
    const T* it = v;                                                                     
    const T* itEnd = it + count;                                                         
    k64f sum = 0;                                                                           
                                                                                            
    if (!v || (count < 1) || !average)                                                      
    {                                                                                       
        return kERROR_PARAMETER;                                                            
    }                                                                                       
                                                                                            
    while (it != itEnd)                                                                     
    {                                                                                       
        sum += (k64f) (*it++);                                                              
    }                                                                                       
                                                                                            
    *average = sum / count;             
    
    return kOK;                                                                             
}

template<typename T> 
kStatus kMathStdev(const T* v, kSize count, k64f* stdev)
{                                                                                           
    const T* it = v;                                                                     
    const T* itEnd = it + count;                                                         
    k64f sum = 0;                                                                           
    k64f sumSq = 0;                                                                         
    k64f num, den, quotient;                                                                
                                                                                            
    if (!v || (count < 2) || !stdev)                                                        
    {                                                                                       
        return kERROR_PARAMETER;                                                            
    }                                                                                       
                                                                                            
    while (it != itEnd)                                                                     
    {                                                                                       
        sumSq += ((k64f) *it) * ((k64f) *it);                                               
        sum   += *it++;                                                                     
    }                                                                                       
                                                                                            
    num = count*sumSq - sum*sum;                                                            
    den = count*(count - 1.0);                                                              
    quotient = num/den;                                                                     
                                                                                            
    if (quotient <= 0)   *stdev = 0;                                                        
    else                 *stdev = sqrt(quotient);                                           
                                                                                            
    return kOK;                                                                             
}

template<typename T> 
kStatus kMathMin(const T* v, kSize count, T* minValue)
{                                                                                           
    kStatus status;                                                                         
    kSize index;                                                                            
                                                                                            
    if (!minValue)                                                                          
    {                                                                                       
        return kERROR_PARAMETER;                                                            
    }                                                                                       
                                                                                            
    status = kMathFindMin(v, count, &index);                                            
    if (!kSuccess(status))                                                                  
    {                                                                                       
        return status;                                                                      
    }                                                                                       
                                                                                            
    *minValue = v[index];                                                                   
    return kOK;                                                                             
}

template<typename T> 
kStatus kMathMax(const T* v, kSize count, T* maxValue)
{                                                                                           
    kStatus status;                                                                         
    kSize index;                                                                            
                                                                                            
    if (!maxValue)                                                                          
    {                                                                                       
        return kERROR_PARAMETER;                                                            
    }                                                                                       
                                                                                            
    status = kMathFindMax(v, count, &index);                                            
    if (!kSuccess(status))                                                                  
    {                                                                                       
        return status;                                                                      
    }                                                                                       
                                                                                            
    *maxValue = v[index];                                                                   

    return kOK;                                                                             
}

template<typename T> 
kStatus kMathCentroid(const T* v, kSize count, k64f* centroid)
{                                                                                           
    const T* it = v;                                                                     
    const T* itEnd = it + count;                                                         
    k64f sum = 0;                                                                           
    k64f wSum = 0;                                                                          
                                                                                            
    if (!v || (count < 1) || !centroid)                                                     
    {                                                                                       
        return kERROR_PARAMETER;                                                            
    }                                                                                       
                                                                                            
    while (it != itEnd)                                                                     
    {                                                                                       
        sum += *it++;                                                                       
        wSum += sum;                                                                        
    }                                                                                       
                                                                                            
    *centroid = (sum != 0) ? (count - wSum/sum) : (count/2.0);                              

    return kOK;                                                                             
}

template<typename T> 
kStatus kMathSet(T* v, kSize count, const T& value)
{                                                                                           
    T* it = v;                                                                           
    T* itEnd = it + count;                                                               
                                                                                            
    if (!v && (count > 0))                                                                  
    {                                                                                       
        return kERROR_PARAMETER;                                                            
    }                                                                                       
                                                                                            
    while (it != itEnd)                                                                     
    {                                                                                       
        *it++ = value;                                                                      
    }                                                                                       
                                                                                            
    return kOK;                                                                             
}

template<typename T> 
kStatus kMathStep(T* v, kSize count, const T& startValue, const T& increment)
{                                                                                           
    T* it = v;                                                                           
    T* itEnd = it + count;                                                               
    T value = startValue;                                                                
                                                                                            
    if (kIsNull(v) && (count > 0))                                                          
    {                                                                                       
        return kERROR_PARAMETER;                                                            
    }                                                                                       
                                                                                            
    while (it != itEnd)                                                                     
    {                                                                                       
        *it++ = value;                                                                      
        value += increment;                                                                 
    }                                                                                       
                                                                                            
    return kOK;                                                                             
}

template<typename T> 
kStatus kMathSpan(T* v, kSize count, const T& startValue, const T& endValue)
{                                                                                           
    kSize i = 0;                                                                            
                                                                                            
    if (kIsNull(v) && (count > 0))                                                          
    {                                                                                       
        return kERROR_PARAMETER;                                                            
    }                                                                                       
    else if (count == 0)                                                                    
    {                                                                                       
        return kOK;                                                                         
    }                                                                                       
    else if (count == 1)                                                                    
    {                                                                                       
        v[0] = endValue;                                                                    
    }                                                                                       
    else                                                                                    
    {                                                                                       
        for (i = 0; i < count; ++i)                                                         
        {                                                                                   
            v[i] = (T) (startValue + (k64f)i*(endValue - startValue)/(count-1.0));            
        }                                                                                   
    }                                                                                       
                                                                                            
    return kOK;                                                                             
}

template<typename T> 
kStatus kMathAbs(const T* vIn, T* vOut, kSize count)
{                                                                                           
    const T* itIn = vIn;                                                                 
    const T* itInEnd = itIn + count;                                                     
    T* itOut = vOut;                                                                     
                                                                                            
    if ((!vIn || !vOut) && (count > 0))                                                     
    {                                                                                       
        return kERROR_PARAMETER;                                                            
    }                                                                                       
                                                                                            
    while (itIn != itInEnd)                                                                 
    {                                                                                       
        *itOut++ = (*itIn >= 0) ? (*itIn) : -(*itIn);                                       
        itIn++;                                                                             
    }                                                                                       
                                                                                            
    return kOK;                                                                             
}

template<typename T, typename Op> 
kStatus kMathApply(const T* vIn, T* vOut, kSize count, const Op& op, const T& value)
{                                                                                           
    const T* itIn = vIn;                                                                 
    const T* itInEnd = itIn + count;                                                     
    T* itOut = vOut;                                                                     
                                                                                            
    if ((!vIn || !vOut) && (count > 0))                                                     
    {                                                                                       
        return kERROR_PARAMETER;                                                            
    }                                                                                       
                                                                                            
    while (itIn != itInEnd)                                                                 
    {                                                                                       
        *itOut++ = op(*itIn++, value);                                            
    }                                                                                       
                                                                                            
    return kOK;                                                                             
}

template<typename T, typename Op> 
kStatus kMathApply(const T* vIn1, const T* vIn2, T* vOut, kSize count, const Op& op)
{                                                                                           
    const T* itIn1 = vIn1;                                                               
    const T* itIn1End = itIn1 + count;                                                   
    const T* itIn2 = vIn2;                                                               
    T* itOut = vOut;                                                                     
                                                                                            
    if ((!vIn1 || !vIn2 || !vOut) && (count > 0))                                           
    {                                                                                       
        return kERROR_PARAMETER;                                                            
    }                                                                                       
                                                                                            
    while (itIn1 != itIn1End)                                                               
    {                                                                                       
        *itOut++ = op(*itIn1++, *itIn2++);                                        
    }                                                                                       
                                                                                            
    return kOK;                                                                             
}

template<typename T> 
kStatus kMathClampC(const T* vIn, T* vOut, kSize count, T minValue, T maxValue)
{                                                                                                       
    const T* itIn = vIn;                                                                             
    const T* itInEnd = itIn + count;                                                                 
    T* itOut = vOut;                                                                                 
                                                                                                        
    if ((!vIn || !vOut) && (count > 0))                                                                 
    {                                                                                                   
        return kERROR_PARAMETER;                                                                        
    }                                                                                                   
                                                                                                        
    while (itIn != itInEnd)                                                                             
    {                                                                                                   
        if      (*itIn < minValue)  *itOut = minValue;                                                  
        else if (*itIn > maxValue)  *itOut = maxValue;                                                  
        else                        *itOut = *itIn;                                                     
        itIn++;                                                                                         
        itOut++;                                                                                        
    }                                                                                                   
                                                                                                        
    return kOK;                                                                                         
}

template<typename T, typename P> 
kStatus kMathReplaceC(const T* vIn, T* vOut, kSize count, const P& pred, T value, T replacement)
{                                                                                                           
    const T* itIn = vIn;                                                                                 
    const T* itInEnd = itIn + count;                                                                     
    T* itOut = vOut;                                                                                     
                                                                                                            
    if ((!vIn || !vOut) && (count > 0))                                                                     
    {                                                                                                       
        return kERROR_PARAMETER;                                                                            
    }                                                                                                       
                                                                                                            
    while (itIn != itInEnd)                                                                                 
    {                                                                                                       
        if (pred(*itIn, value))     *itOut = replacement;                                           
        else                        *itOut = *itIn;                                                 
        itIn++;                                                                                             
        itOut++;                                                                                            
    }                                                                                                       
                                                                                                            
    return kOK;                                                                                             
}

template<typename T, typename P> 
kStatus kMathCompareC(const T* vIn, kBool* vOut, kSize count, const P& pred, T value)
{                                                                                           
    const T* itIn = vIn;                                                                 
    const T* itInEnd = itIn + count;                                                     
    kBool* itOut = vOut;                                                                    
                                                                                            
    if ((!vIn || !vOut) && (count > 0))                                                     
    {                                                                                       
        return kERROR_PARAMETER;                                                            
    }                                                                                       
                                                                                            
    while (itIn != itInEnd)                                                                 
    {                                                                                       
        *itOut++ = pred(*itIn++, value);                                          
    }                                                                                       
                                                                                            
    return kOK;                                                                             
}

template<typename T> 
kStatus kMathMovingAvg(const T* vIn, T* vOut, kSize count, kSize window)
{                                                                                           
    const T* itIn = vIn;                                                                 
    const T* itInEnd = itIn + count;                                                     
    T* itOut = vOut;                                                                     
    const T *windowBegin, *windowEnd, *windowReader, *windowBeginEnd;                    
    T sum, average;                                                                      
                                                                                            
    if (!vIn || !vOut || (window < 1) || (count < window))                                  
    {                                                                                       
        return kERROR_PARAMETER;                                                            
    }                                                                                       
                                                                                            
    sum = 0;                                                                                
    windowBegin = itIn;                                                                     
    windowEnd = itIn + window;                                                              
    windowReader = windowBegin;                                                             
    while (windowReader != windowEnd)                                                       
    {                                                                                       
        sum += *windowReader++;                                                             
    }                                                                                       
                                                                                            
    windowBeginEnd = itInEnd - window;                                                      
    while (windowBegin != windowBeginEnd)                                                   
    {                                                                                       
        average = sum/(T)window;                                                         
        sum -= *windowBegin++;                                                              
        sum += *windowEnd++;                                                                
        *itOut++ = average;                                                                 
    }                                                                                       
                                                                                            
    *itOut = sum/(T)window;                                                              
                                                                                            
    return kOK;                                                                             
}

template<typename T>
kStatus kMath_Gcd(T a, T b, T& out)
{                                                                                           
    T mod;                                                                               
                                                                                            
    /* Iterative version of */                                                              
    /* f(a, b) {return b == 0 ? a : f(b, a % b);} */                                        
    while (kTRUE)                                                                           
    {                                                                                       
        if (b == 0)                                                                         
        {                                                                                   
            break;                                                                          
        }                                                                                   
                                                                                            
        mod = a % b;                                                                        
        a = b;                                                                              
        b = mod;                                                                            
    }                                                                                       
                                                                                            
    out = kMath_Abs_(a);                                                                   
                                                                                            
    return kOK;                                                                             
}

template<typename T>
kStatus kMath_Lcm(const T& a, const T& b, T& out)
{                                                                                           
    T gcd;                                                                               
                                                                                            
    /* a = b = 0 gives 0 for GCD; handle this as a special case. */                         
    if (a == 0 && b == 0)                                                                   
    {                                                                                       
        out = 0;                                                                           
                                                                                            
        return kOK;                                                                         
    }                                                                                       
                                                                                            
    kCheck(kMath_Gcd(a, b, gcd));                                                       
                                                                                            
    out = kMath_Abs_((a * b) / gcd);                                                       
                                                                                            
    return kOK;                                                                             
}

/* Exports */

kFx(kStatus) kMath_FindFirst32s(const k32s* v, kSize count, kComparison comparison, k32s value, kSize* index)
{
    switch(comparison)
    {
        case kCOMPARISON_EQ:    return kMathFindFirst(v, count, kIsEq <k32s>(), value, index);
        case kCOMPARISON_NEQ:   return kMathFindFirst(v, count, kIsNeq<k32s>(), value, index);
        case kCOMPARISON_LT:    return kMathFindFirst(v, count, kIsLt <k32s>(), value, index);
        case kCOMPARISON_LTE:   return kMathFindFirst(v, count, kIsLte<k32s>(), value, index);
        case kCOMPARISON_GT:    return kMathFindFirst(v, count, kIsGt <k32s>(), value, index);
        case kCOMPARISON_GTE:   return kMathFindFirst(v, count, kIsGte<k32s>(), value, index);
        default:                return kERROR_PARAMETER;
    }
}

kFx(kStatus) kMath_FindFirst64f(const k64f* v, kSize count, kComparison comparison, k64f value, kSize* index)
{
    switch(comparison)
    {
        case kCOMPARISON_EQ:    return kMathFindFirst(v, count, kIsEq <k64f>(), value, index);
        case kCOMPARISON_NEQ:   return kMathFindFirst(v, count, kIsNeq<k64f>(), value, index);
        case kCOMPARISON_LT:    return kMathFindFirst(v, count, kIsLt <k64f>(), value, index);
        case kCOMPARISON_LTE:   return kMathFindFirst(v, count, kIsLte<k64f>(), value, index);
        case kCOMPARISON_GT:    return kMathFindFirst(v, count, kIsGt <k64f>(), value, index);
        case kCOMPARISON_GTE:   return kMathFindFirst(v, count, kIsGte<k64f>(), value, index);
        default:                return kERROR_PARAMETER;
    }
}

kFx(kStatus) kMath_FindLast32s(const k32s* v, kSize count, kComparison comparison, k32s value, kSize* index)
{
    switch(comparison)
    {
        case kCOMPARISON_EQ:    return kMathFindLast(v, count, kIsEq <k32s>(), value, index);
        case kCOMPARISON_NEQ:   return kMathFindLast(v, count, kIsNeq<k32s>(), value, index);
        case kCOMPARISON_LT:    return kMathFindLast(v, count, kIsLt <k32s>(), value, index);
        case kCOMPARISON_LTE:   return kMathFindLast(v, count, kIsLte<k32s>(), value, index);
        case kCOMPARISON_GT:    return kMathFindLast(v, count, kIsGt <k32s>(), value, index);
        case kCOMPARISON_GTE:   return kMathFindLast(v, count, kIsGte<k32s>(), value, index);
        default:                return kERROR_PARAMETER;
    }
}

kFx(kStatus) kMath_FindLast64f(const k64f* v, kSize count, kComparison comparison, k64f value, kSize* index)
{
    switch(comparison)
    {
        case kCOMPARISON_EQ:    return kMathFindLast(v, count, kIsEq <k64f>(), value, index);
        case kCOMPARISON_NEQ:   return kMathFindLast(v, count, kIsNeq<k64f>(), value, index);
        case kCOMPARISON_LT:    return kMathFindLast(v, count, kIsLt <k64f>(), value, index);
        case kCOMPARISON_LTE:   return kMathFindLast(v, count, kIsLte<k64f>(), value, index);
        case kCOMPARISON_GT:    return kMathFindLast(v, count, kIsGt <k64f>(), value, index);
        case kCOMPARISON_GTE:   return kMathFindLast(v, count, kIsGte<k64f>(), value, index);
        default:                return kERROR_PARAMETER;
    }
}

kFx(kStatus) kMath_FindMin8u(const k8u* v, kSize count, kSize* index)
{
    return kMathFindMin(v, count, index);
}

kFx(kStatus) kMath_FindMin32u(const k32u* v, kSize count, kSize* index)
{
    return kMathFindMin(v, count, index);
}

kFx(kStatus) kMath_FindMin32s(const k32s* v, kSize count, kSize* index)
{
    return kMathFindMin(v, count, index);
}

kFx(kStatus) kMath_FindMin64u(const k64u* v, kSize count, kSize* index)
{
    return kMathFindMin(v, count, index);
}

kFx(kStatus) kMath_FindMin64f(const k64f* v, kSize count, kSize* index)
{
    return kMathFindMin(v, count, index);
}

kFx(kStatus) kMath_FindMax8u(const k8u* v, kSize count, kSize* index)
{
    return kMathFindMax(v, count, index);
}

kFx(kStatus) kMath_FindMax32u(const k32u* v, kSize count, kSize* index)
{
    return kMathFindMax(v, count, index);
}

kFx(kStatus) kMath_FindMax32s(const k32s* v, kSize count, kSize* index)
{
    return kMathFindMax(v, count, index);
}

kFx(kStatus) kMath_FindMax64u(const k64u* v, kSize count, kSize* index)
{
    return kMathFindMax(v, count, index);
}

kFx(kStatus) kMath_FindMax64f(const k64f* v, kSize count, kSize* index)
{
    return kMathFindMax(v, count, index);
}

kFx(kStatus) kMath_Sum32s(const k32s* v, kSize count, k64s* sum)
{
    return kMathSum(v, count, sum);
}

kFx(kStatus) kMath_Sum64u(const k64u* v, kSize count, k64u* sum)
{
    return kMathSum(v, count, sum);
}

kFx(kStatus) kMath_Sum64f(const k64f* v, kSize count, k64f* sum)
{
    return kMathSum(v, count, sum);
}

kFx(kStatus) kMath_Average8u(const k8u* v, kSize count, k64f* average)
{
    return kMathAverage(v, count, average);
}

kFx(kStatus) kMath_Average32s(const k32s* v, kSize count, k64f* average)
{
    return kMathAverage(v, count, average);
}

kFx(kStatus) kMath_Average64f(const k64f* v, kSize count, k64f* average)
{
    return kMathAverage(v, count, average);
}

kFx(kStatus) kMath_Stdev8u(const k8u* v, kSize count, k64f* stdev)
{
    return kMathStdev(v, count, stdev);
}

kFx(kStatus) kMath_Stdev32s(const k32s* v, kSize count, k64f* stdev)
{
    return kMathStdev(v, count, stdev);
}

kFx(kStatus) kMath_Stdev64f(const k64f* v, kSize count, k64f* stdev)
{
    return kMathStdev(v, count, stdev);
}

kFx(kStatus) kMath_Min8u(const k8u* v, kSize count, k8u* minValue)
{
    return kMathMin(v, count, minValue);
}

kFx(kStatus) kMath_Min32s(const k32s* v, kSize count, k32s* minValue)
{
    return kMathMin(v, count, minValue);
}

kFx(kStatus) kMath_Min32u(const k32u* v, kSize count, k32u* minValue)
{
    return kMathMin(v, count, minValue);
}

kFx(kStatus) kMath_Min64u(const k64u* v, kSize count, k64u* minValue)
{
    return kMathMin(v, count, minValue);
}

kFx(kStatus) kMath_Min64f(const k64f* v, kSize count, k64f* minValue)
{
    return kMathMin(v, count, minValue);
}

kFx(kStatus) kMath_Max8u(const k8u* v, kSize count, k8u* maxValue)
{
    return kMathMax(v, count, maxValue);
}

kFx(kStatus) kMath_Max32u(const k32u* v, kSize count, k32u* maxValue)
{
    return kMathMax(v, count, maxValue);
}

kFx(kStatus) kMath_Max32s(const k32s* v, kSize count, k32s* maxValue)
{
    return kMathMax(v, count, maxValue);
}

kFx(kStatus) kMath_Max64u(const k64u* v, kSize count, k64u* maxValue)
{
    return kMathMax(v, count, maxValue);
}

kFx(kStatus) kMath_Max64f(const k64f* v, kSize count, k64f* maxValue)
{
    return kMathMax(v, count, maxValue);
}


kFx(kStatus) kMath_Centroid32s(const k32s* v, kSize count, k64f* centroid)
{
    return kMathCentroid(v, count, centroid);
}

kFx(kStatus) kMath_Centroid64f(const k64f* v, kSize count, k64f* centroid)
{
    return kMathCentroid(v, count, centroid);
}

kFx(kStatus) kMath_Set32s(k32s* v, kSize count, k32s value)
{
    return kMathSet(v, count, value);
}

kFx(kStatus) kMath_Set64f(k64f* v, kSize count, k64f value)
{
    return kMathSet(v, count, value);
}

kFx(kStatus) kMath_Step32s(k32s* v, kSize count, k32s startValue, k32s increment)
{
    return kMathStep(v, count, startValue, increment);
}

kFx(kStatus) kMath_Step64f(k64f* v, kSize count, k64f startValue, k64f increment)
{
    return kMathStep(v, count, startValue, increment); 
}

kFx(kStatus) kMath_Span32s(k32s* v, kSize count, k32s startValue, k32s endValue)
{
    return kMathSpan(v, count, startValue, endValue);
}

kFx(kStatus) kMath_Span64f(k64f* v, kSize count, k64f startValue, k64f endValue)
{
    return kMathSpan(v, count, startValue, endValue);
}

kFx(kStatus) kMath_Abs32s(const k32s* vIn, k32s* vOut, kSize count)
{
    return kMathAbs(vIn, vOut, count);
}

kFx(kStatus) kMath_Abs64f(const k64f* vIn, k64f* vOut, kSize count)
{
    return kMathAbs(vIn, vOut, count);
}

kFx(kStatus) kMath_AddC32s(const k32s* vIn, k32s* vOut, kSize count, k32s value)
{
    return kMathApply(vIn, vOut, count, kAdd<k32s>(), value);
}

kFx(kStatus) kMath_AddC64f(const k64f* vIn, k64f* vOut, kSize count, k64f value)
{
    return kMathApply(vIn, vOut, count, kAdd<k64f>(), value);
}

kFx(kStatus) kMath_SubC32s(const k32s* vIn, k32s* vOut, kSize count, k32s value)
{
    return kMathApply(vIn, vOut, count, kSubtract<k32s>(), value);
}

kFx(kStatus) kMath_SubC64f(const k64f* vIn, k64f* vOut, kSize count, k64f value)
{
    return kMathApply(vIn, vOut, count, kSubtract<k64f>(), value);
}

kFx(kStatus) kMath_MulC32s(const k32s* vIn, k32s* vOut, kSize count, k32s value)
{
    return kMathApply(vIn, vOut, count, kMultiply<k32s>(), value);
}

kFx(kStatus) kMath_MulC64f(const k64f* vIn, k64f* vOut, kSize count, k64f value)
{
    return kMathApply(vIn, vOut, count, kMultiply<k64f>(), value);
}

kFx(kStatus) kMath_DivC32s(const k32s* vIn, k32s* vOut, kSize count, k32s value)
{
    return kMathApply(vIn, vOut, count, kDivide<k32s>(), value);
}

kFx(kStatus) kMath_DivC64f(const k64f* vIn, k64f* vOut, kSize count, k64f value)
{
    return kMathApply(vIn, vOut, count, kDivide<k64f>(), value);
}

kFx(kStatus) kMath_ClampC32s(const k32s* vIn, k32s* vOut, kSize count, k32s minValue, k32s maxValue)
{
    return kMathClampC(vIn, vOut, count, minValue, maxValue);
}

kFx(kStatus) kMath_ClampC64f(const k64f* vIn, k64f* vOut, kSize count, k64f minValue, k64f maxValue)
{
    return kMathClampC(vIn, vOut, count, minValue, maxValue);
}

kFx(kStatus) kMath_ReplaceC32s(const k32s* vIn, k32s* vOut, kSize count, kComparison comparison, k32s value, k32s replacement)
{
    switch(comparison)
    {
        case kCOMPARISON_EQ:    return kMathReplaceC(vIn, vOut, count, kIsEq <k32s>(), value, replacement);
        case kCOMPARISON_NEQ:   return kMathReplaceC(vIn, vOut, count, kIsNeq<k32s>(), value, replacement);
        case kCOMPARISON_LT:    return kMathReplaceC(vIn, vOut, count, kIsLt <k32s>(), value, replacement);
        case kCOMPARISON_LTE:   return kMathReplaceC(vIn, vOut, count, kIsLte<k32s>(), value, replacement);
        case kCOMPARISON_GT:    return kMathReplaceC(vIn, vOut, count, kIsGt <k32s>(), value, replacement);
        case kCOMPARISON_GTE:   return kMathReplaceC(vIn, vOut, count, kIsGte<k32s>(), value, replacement);
        default:                return kERROR_PARAMETER;
    }
}

kFx(kStatus) kMath_ReplaceC64f(const k64f* vIn, k64f* vOut, kSize count, kComparison comparison, k64f value, k64f replacement)
{
    switch(comparison)
    {
        case kCOMPARISON_EQ:    return kMathReplaceC(vIn, vOut, count, kIsEq <k64f>(), value, replacement);
        case kCOMPARISON_NEQ:   return kMathReplaceC(vIn, vOut, count, kIsNeq<k64f>(), value, replacement);
        case kCOMPARISON_LT:    return kMathReplaceC(vIn, vOut, count, kIsLt <k64f>(), value, replacement);
        case kCOMPARISON_LTE:   return kMathReplaceC(vIn, vOut, count, kIsLte<k64f>(), value, replacement);
        case kCOMPARISON_GT:    return kMathReplaceC(vIn, vOut, count, kIsGt <k64f>(), value, replacement);
        case kCOMPARISON_GTE:   return kMathReplaceC(vIn, vOut, count, kIsGte<k64f>(), value, replacement);
        default:                return kERROR_PARAMETER;
    }
}

kFx(kStatus) kMath_CompareC32s(const k32s* vIn, kBool* vOut, kSize count, kComparison comparison, k32s value)
{
    switch(comparison)
    {
        case kCOMPARISON_EQ:    return kMathCompareC(vIn, vOut, count, kIsEq <k32s>(), value);
        case kCOMPARISON_NEQ:   return kMathCompareC(vIn, vOut, count, kIsNeq<k32s>(), value);
        case kCOMPARISON_LT:    return kMathCompareC(vIn, vOut, count, kIsLt <k32s>(), value);
        case kCOMPARISON_LTE:   return kMathCompareC(vIn, vOut, count, kIsLte<k32s>(), value);
        case kCOMPARISON_GT:    return kMathCompareC(vIn, vOut, count, kIsGt <k32s>(), value);
        case kCOMPARISON_GTE:   return kMathCompareC(vIn, vOut, count, kIsGte<k32s>(), value);
        default:                return kERROR_PARAMETER;
    }
}

kFx(kStatus) kMath_CompareC64f(const k64f* vIn, kBool* vOut, kSize count, kComparison comparison, k64f value)
{
    switch(comparison)
    {
        case kCOMPARISON_EQ:    return kMathCompareC(vIn, vOut, count, kIsEq <k64f>(), value);
        case kCOMPARISON_NEQ:   return kMathCompareC(vIn, vOut, count, kIsNeq<k64f>(), value);
        case kCOMPARISON_LT:    return kMathCompareC(vIn, vOut, count, kIsLt <k64f>(), value);
        case kCOMPARISON_LTE:   return kMathCompareC(vIn, vOut, count, kIsLte<k64f>(), value);
        case kCOMPARISON_GT:    return kMathCompareC(vIn, vOut, count, kIsGt <k64f>(), value);
        case kCOMPARISON_GTE:   return kMathCompareC(vIn, vOut, count, kIsGte<k64f>(), value);
        default:                return kERROR_PARAMETER;
    }
}

kFx(kStatus) kMath_Add32s(const k32s* vIn1, const k32s* vIn2, k32s* vOut, kSize count)
{
    return kMathApply(vIn1, vIn2, vOut, count, kAdd<k32s>());
}

kFx(kStatus) kMath_Add64f(const k64f* vIn1, const k64f* vIn2, k64f* vOut, kSize count)
{
    return kMathApply(vIn1, vIn2, vOut, count, kAdd<k64f>());
}

kFx(kStatus) kMath_Sub32s(const k32s* vIn1, const k32s* vIn2, k32s* vOut, kSize count)
{
    return kMathApply(vIn1, vIn2, vOut, count, kSubtract<k32s>());
}

kFx(kStatus) kMath_Sub64f(const k64f* vIn1, const k64f* vIn2, k64f* vOut, kSize count)
{
    return kMathApply(vIn1, vIn2, vOut, count, kSubtract<k64f>());
}

kFx(kStatus) kMath_Mul32s(const k32s* vIn1, const k32s* vIn2, k32s* vOut, kSize count)
{
    return kMathApply(vIn1, vIn2, vOut, count, kMultiply<k32s>());
}

kFx(kStatus) kMath_Mul64f(const k64f* vIn1, const k64f* vIn2, k64f* vOut, kSize count)
{
    return kMathApply(vIn1, vIn2, vOut, count, kMultiply<k64f>());
}

kFx(kStatus) kMath_Div32s(const k32s* vIn1, const k32s* vIn2, k32s* vOut, kSize count)
{
    return kMathApply(vIn1, vIn2, vOut, count, kDivide<k32s>());
}

kFx(kStatus) kMath_Div64f(const k64f* vIn1, const k64f* vIn2, k64f* vOut, kSize count)
{
    return kMathApply(vIn1, vIn2, vOut, count, kDivide<k64f>());
}

kFx(kStatus) kMath_MovingAvg32s(const k32s* vIn, k32s* vOut, kSize count, kSize window)
{
    return kMathMovingAvg(vIn, vOut, count, window);
}

kFx(kStatus) kMath_MovingAvg64f(const k64f* vIn, k64f* vOut, kSize count, kSize window)
{
    return kMathMovingAvg(vIn, vOut, count, window);
}

kFx(kStatus) kMath_Gcd32s(k32s a, k32s b, k32s* result)
{
    return kMath_Gcd(a, b, *result);
}

kFx(kStatus) kMath_Lcm32s(k32s a, k32s b, k32s* result)
{
    return kMath_Lcm(a, b, *result);
}

/* Adapted from: http://stackoverflow.com/questions/3272424/compute-fast-log-base-2-ceiling */
kFx(k32u) kMath_Log2Ceil32u(k32u a)
{
    static const k32u t[5] = { 0xFFFF0000u, 0x0000FF00u, 0x000000F0u, 0x0000000Cu, 0x00000002u }; 
    k32u y = ((a & (a-1u)) == 0u) ? 0u : 1u; 
    k32u j = 16; 
    k32u i, k; 

    for (i = 0; i < kCountOf(t); ++i)
    {
        k = ((a & t[i]) == 0) ? 0 : j; 
        y += k; 
        a >>= k; 
        j >>= 1; 
    }

    return y; 
}

kFx(k64f) kMath_Round64f(k64f a)
{
    return (a >= 0.0) ? floor(a + 0.5) : ceil(a - 0.5);
}
