//
// KMath.h
// 
// Copyright (C) 2014-2021 by LMI Technologies Inc.
// Licensed under the MIT License.
// Redistributed files must retain the above copyright notice.
// 
#ifndef K_API_NET_MATH_H
#define K_API_NET_MATH_H

#include <kApi/Data/kMath.h>
#include "kApiNet/KAlloc.h"

namespace Lmi3d
{
    namespace Zen 
    {       
        namespace Data
        {
            /// <summary>Collection of mathematical utility methods.</summary>
            /// 
            /// <remarks>The methods in this class are designed to work with native arrays, passed via pointers.
            ///
            /// <para>Default KRefStyle: None</para>
            ///</remarks>
            public ref class KMath abstract : public KObject
            {
                KDeclareNoneClass(KMath, kMath)

            public:

                /// <summary>Mathematical constant pi.</summary>
                literal k64f Pi = (3.1415926535897932384626433832795); 

                /// <summary>Mathematical constant e.</summary>
                literal k64f E = (2.7182818284590452353602874713527); 

                /// <summary>Square root of two.</summary>
                literal k64f Sqrt2 = (1.4142135623730950488016887242097); 

                /// <summary>Square root of three.</summary>
                literal k64f Sqrt3 = (1.7320508075688772935274463415059);

                /// <summary> Reports whether two single-precision values are approximately equal. </summary> 
                static kBool NearlyEquals(k32f a, k32f b)
                {
                    return kMath_NearlyEquals32f(a, b);
                }

                /// <summary> Reports whether two double-precision values are approximately equal. </summary> 
                static kBool NearlyEquals(k64f a, k64f b)
                {
                    return kMath_NearlyEquals64f(a, b);
                }

                /// <summary> Converts radians to degrees. </summary> 
                static k64f RadToDeg(k64f radians)
                {
                    return kMath_RadToDeg_(radians);
                }

                /// <summary> Converts degrees to radians. </summary> 
                static k64f DegToRad(k64f degrees)
                {
                    return kMath_DegToRad_(degrees);
                }

                /// <summary> Returns the common residue of a and b. </summary> 
                static k32s CommonResidue(k32s a, k32s b)
                {
                    return kMath_Common_Residue_(a, b);
                }

                /// <summary> Returns a value limited to the given range. </summary>               
                static k32s Clamp(k32s v, k32s min, k32s max)
                {
                    return kMath_Clamp_(v, min, max);
                }

                /// <summary>Compares each value in a numerical array with a specified value and determines the index of the first match.</summary>
                /// 
                /// <param name="v">Array of Int32 values.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="comparison">Comparator that will be used to determine a match.</param>
                /// <param name="value">Value for comparison.</param>
                /// <param name="index">Receives the index of the first match (or count, if no match is found).</param>
                /// <returns>true if a match was found.</returns>
                static bool TryFindFirst32s(IntPtr v, k64s count, KComparison comparison, k32s value, [Out] k64s% index)
                {
                    kSize idx = 0;
                    kStatus result = kMath_FindFirst32s((k32s*)v.ToPointer(), (kSize)count, comparison, value, &idx);
                   
                    index = (k64s)idx;

                    return KToBool(kSuccess(result)); 
                }

                /// <summary>Compares each value in a numerical array with a specified value and determines the index of the first match.</summary>
                /// 
                /// <param name="v">Array of Double values.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="comparison">Comparator used to determine a match.</param>
                /// <param name="value">Value for comparison.</param>
                /// <param name="index">Receives the index of the first match (or count, if no match is found).</param>
                /// <returns>true if a match was found.</returns>
                static bool TryFindFirst64f(IntPtr v, k64s count, KComparison comparison, k64f value, [Out] k64s% index)
                {
                    kSize idx = 0;
                    kStatus result = kMath_FindFirst64f((k64f*)v.ToPointer(), (kSize)count, comparison, value, &idx);

                    index = (k64s)idx;

                    return KToBool(kSuccess(result));
                }

                /// <summary>Compares each value in a numerical array with a specified value and determines the index of the last match.</summary>
                /// 
                /// <param name="v">Array of Int32 values.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="comparison">Comparator used to determine a match.</param>
                /// <param name="value">Value for comparison.</param>
                /// <param name="index">Receives the index of the last match (or count, if no match is found).</param>
                /// <returns>true if a match was found.</returns>
                static bool TryFindLast32s(IntPtr v, k64s count, KComparison comparison, k32s value, [Out] k64s% index)
                {
                    kSize idx = 0;
                    kStatus result = kMath_FindLast32s((k32s*)v.ToPointer(), (kSize)count, comparison, value, &idx);

                    index = (k64s)idx;

                    return KToBool(kSuccess(result));
                }

                /// <summary>Compares each value in a numerical array with a specified value and determines the index of the last match.</summary>
                /// 
                /// <param name="v">Array of Double values.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="comparison">Comparator that will be used to determine a match.</param>
                /// <param name="value">Value for comparison.</param>
                /// <param name="index">Receives the index of the last match (or count, if no match is found).</param>
                /// <returns>true if a match was found.</returns>
                static bool TryFindLast64f(IntPtr v, k64s count, KComparison comparison, k64f value, [Out] k64s% index)
                {
                    kSize idx = 0;
                    kStatus result = kMath_FindLast64f((k64f*)v.ToPointer(), (kSize)count, comparison, value, &idx);

                    index = (k64s)idx;

                    return KToBool(kSuccess(result));
                }

                /// <summary>Finds the index of the minimum value within a numerical array.</summary>
                /// 
                /// <param name="v">Array of Int32 values.</param>
                /// <param name="count">Count of values.</param>
                /// <returns>Index of the minimum value.</returns>
                static k64s FindMin32s(IntPtr v, k64s count)
                {
                    kSize index; 

                    KCheck(kMath_FindMin32s((k32s*)v.ToPointer(), (kSize)count, &index)); 

                    return (k64s)index; 
                }

                /// <summary>Finds the index of the minimum value within a numerical array.</summary>
                /// 
                /// <param name="v">Array of Double values.</param>
                /// <param name="count">Count of values.</param>
                /// <returns>Index of the minimum value.</returns>
                static k64s FindMin64f(IntPtr v, k64s count)
                {
                    kSize index;

                    KCheck(kMath_FindMin64f((k64f*)v.ToPointer(), (kSize)count, &index));

                    return (k64s)index;
                }

                /// <summary>Finds the index of the maximum value within a numerical array.</summary>
                /// 
                /// <param name="v">Array of Int32 values.</param>
                /// <param name="count">Count of values.</param>
                /// <returns>Index of the maximum value.</returns>
                static k64s FindMax32s(IntPtr v, k64s count)
                {
                    kSize index;

                    KCheck(kMath_FindMax32s((k32s*)v.ToPointer(), (kSize)count, &index));

                    return (k64s)index;
                }

                /// <summary>Finds the index of the maximum value within a numerical array.</summary>
                /// 
                /// <param name="v">Array of Double values.</param>
                /// <param name="count">Count of values.</param>
                /// <returns>Index of the maximum value.</returns>
                static k64s FindMax64f(IntPtr v, k64s count)
                {
                    kSize index;

                    KCheck(kMath_FindMax64f((k64f*)v.ToPointer(), (kSize)count, &index));

                    return (k64s)index;
                }

                /// <summary>Calculates the sum of a numerical array.</summary>
                /// 
                /// <param name="v">Array of Int32 values.</param>
                /// <param name="count">Count of values.</param>
                /// <returns>Sum.</returns>
                static k64s Sum32s(IntPtr v, k64s count)
                {
                    k64s sum; 

                    KCheck(kMath_Sum32s((k32s*)v.ToPointer(), (kSize)count, &sum)); 

                    return sum; 
                }

                /// <summary>Calculates the sum of a numerical array.</summary>
                /// 
                /// <param name="v">Array of Double values.</param>
                /// <param name="count">Count of values.</param>
                /// <returns>Sum.</returns>
                static k64f Sum64f(IntPtr v, k64s count)
                {
                    k64f sum;

                    KCheck(kMath_Sum64f((k64f*)v.ToPointer(), (kSize)count, &sum));

                    return sum;
                }

                /// <averagemary>Calculates the average for a numerical array.</averagemary>
                /// 
                /// <param name="v">Array of Int32 values.</param>
                /// <param name="count">Count of values.</param>
                /// <returns>Average.</returns>
                static k64f Average32s(IntPtr v, k64s count)
                {
                    k64f average;

                    KCheck(kMath_Average32s((k32s*)v.ToPointer(), (kSize)count, &average));

                    return average;
                }

                /// <averagemary>Calculates the average for a numerical array.</averagemary>
                /// 
                /// <param name="v">Array of Double values.</param>
                /// <param name="count">Count of values.</param>
                /// <returns>Average.</returns>
                static k64f Average64f(IntPtr v, k64s count)
                {
                    k64f average;

                    KCheck(kMath_Average64f((k64f*)v.ToPointer(), (kSize)count, &average));

                    return average;
                }

                /// <stdevmary>Calculates the standard deviation of a numerical array.</stdevmary>
                /// 
                /// <param name="v">Array of Int32 values.</param>
                /// <param name="count">Count of values.</param>
                /// <returns>Standard deviation.</returns>
                static k64f Stdev32s(IntPtr v, k64s count)
                {
                    k64f stdev;

                    KCheck(kMath_Stdev32s((k32s*)v.ToPointer(), (kSize)count, &stdev));

                    return stdev;
                }

                /// <stdevmary>Calculates the standard deviation of a numerical array.</stdevmary>
                /// 
                /// <param name="v">Array of Double values.</param>
                /// <param name="count">Count of values.</param>
                /// <returns>Standard deviation.</returns>
                static k64f Stdev64f(IntPtr v, k64s count)
                {
                    k64f stdev;

                    KCheck(kMath_Stdev64f((k64f*)v.ToPointer(), (kSize)count, &stdev));

                    return stdev;
                }

                /// <summary>Reports the minimum value in a numerical array.</summary>
                /// 
                /// <param name="v">Array of Int32 values.</param>
                /// <param name="count">Count of values.</param>
                /// <returns>Minimum value.</returns>
                static k32s Min32s(IntPtr v, k64s count)
                {
                    k32s result;

                    KCheck(kMath_Min32s((k32s*)v.ToPointer(), (kSize)count, &result));

                    return result;
                }

                /// <summary>Reports the minimum value in a numerical array.</summary>
                /// 
                /// <param name="v">Array of Double values.</param>
                /// <param name="count">Count of values.</param>
                /// <returns>Minimum value.</returns>
                static k64f Min64f(IntPtr v, k64s count)
                {
                    k64f result;

                    KCheck(kMath_Min64f((k64f*)v.ToPointer(), (kSize)count, &result));

                    return result;
                }

                /// <summary>Reports the maximum value in a numerical array.</summary>
                /// 
                /// <param name="v">Array of Int32 values.</param>
                /// <param name="count">Count of values.</param>
                /// <returns>Maximum value.</returns>
                static k32s Max32s(IntPtr v, k64s count)
                {
                    k32s result;

                    KCheck(kMath_Max32s((k32s*)v.ToPointer(), (kSize)count, &result));

                    return result;
                }

                /// <summary>Reports the maximum value in a numerical array.</summary>
                /// 
                /// <param name="v">Array of Double values.</param>
                /// <param name="count">Count of values.</param>
                /// <returns>Maximum value.</returns>
                static k64f Max64f(IntPtr v, k64s count)
                {
                    k64f result;

                    KCheck(kMath_Max64f((k64f*)v.ToPointer(), (kSize)count, &result));

                    return result;
                }
              
                /// <summary>Calculates the center of gravity for a numerical array.</summary>
                /// 
                /// <param name="v">Array of Int32 values.</param>
                /// <param name="count">Count of values.</param>
                /// <returns>The centroid.</returns>
                static k64f Centroid32s(IntPtr v, k64s count)
                {
                    k64f result;

                    KCheck(kMath_Centroid32s((k32s*)v.ToPointer(), (kSize)count, &result));

                    return result;
                }

                /// <summary>Calculates the center of gravity for a numerical array.</summary>
                /// 
                /// <param name="v">Array of Double values.</param>
                /// <param name="count">Count of values.</param>
                /// <returns>The centroid.</returns>
                static k64f Centroid64f(IntPtr v, k64s count)
                {
                    k64f result;

                    KCheck(kMath_Centroid64f((k64f*)v.ToPointer(), (kSize)count, &result));

                    return result;
                }

                /// <summary>Sets all values in a numerical array to the given value.</summary>
                /// 
                /// <param name="v">Array of Int32 values.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="value">Value to set.</param>
                static void Set32s(IntPtr v, k64s count, k32s value)
                {
                    KCheck(kMath_Set32s((k32s*)v.ToPointer(), (kSize)count, value)); 
                }

                /// <summary>Sets all values in a numerical array to the given value.</summary>
                /// 
                /// <param name="v">Array of Double values.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="value">Value to set.</param>
                static void Set64f(IntPtr v, k64s count, k64f value)
                {
                    KCheck(kMath_Set64f((k64f*)v.ToPointer(), (kSize)count, value)); 
                }

                /// <summary>Sets values in a numerical array to increment from the specified starting value.</summary>
                /// 
                /// <param name="v">Array of Int32 values.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="startValue">First value.</param>
                /// <param name="increment">Increment amount.</param>
                static void Step32s(IntPtr v, k64s count, k32s startValue, k32s increment)
                {
                    KCheck(kMath_Step32s((k32s*)v.ToPointer(), (kSize)count, startValue, increment)); 
                }
            
                /// <summary>Sets values in a numerical array to increment from the specified starting value.</summary>
                /// 
                /// <param name="v">Array of Double values.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="startValue">First value.</param>
                /// <param name="increment">Increment amount.</param>
                static void Step64f(IntPtr v, k64s count, k64f startValue, k64f increment)
                {
                    KCheck(kMath_Step64f((k64f*)v.ToPointer(), (kSize)count, startValue, increment));
                }

                /// <summary>Sets values in a numerical array to step between the specified start and end values.</summary>
                /// 
                /// <param name="v">Array of Int32 values.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="startValue">First value.</param>
                /// <param name="endValue">Last value.</param>
                static void Span32s(IntPtr v, k64s count, k32s startValue, k32s endValue)
                {
                    KCheck(kMath_Span32s((k32s*)v.ToPointer(), (kSize)count, startValue, endValue));
                }

                /// <summary>Sets values in a numerical array to step between the specified start and end values.</summary>
                /// 
                /// <param name="v">Array of Double values.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="startValue">First value.</param>
                /// <param name="endValue">Last value.</param>
                static void Span64f(IntPtr v, k64s count, k64f startValue, k64f endValue)
                {
                    KCheck(kMath_Span64f((k64f*)v.ToPointer(), (kSize)count, startValue, endValue));
                }

                /// <summary>Calculates the absolute value of each element in an input array and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn">Input Int32 array.</param>
                /// <param name="vOut">Output Int32 array.</param>
                /// <param name="count">Count of values.</param>
                static void Abs32s(IntPtr vIn, IntPtr vOut, k64s count)
                {
                    KCheck(kMath_Abs32s((k32s*)vIn.ToPointer(), (k32s*)vOut.ToPointer(), (kSize)count));
                }

                /// <summary>Calculates the absolute value of each element in an input array and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn">Input Double array.</param>
                /// <param name="vOut">Output Double array.</param>
                /// <param name="count">Count of values.</param>
                static void Abs64f(IntPtr vIn, IntPtr vOut, k64s count)
                {
                    KCheck(kMath_Abs64f((k64f*)vIn.ToPointer(), (k64f*)vOut.ToPointer(), (kSize)count));
                }

                /// <summary>Adds a constant to each element in an input array and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn">Input Int32 array.</param>
                /// <param name="vOut">Output Int32 array.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="value">Constant value.</param>
                static void AddC32s(IntPtr vIn, IntPtr vOut, k64s count, k32s value)
                {
                    KCheck(kMath_AddC32s((k32s*)vIn.ToPointer(), (k32s*)vOut.ToPointer(), (kSize)count, value));
                }

                /// <summary>Adds a constant to each element in an input array and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn">Input Double array.</param>
                /// <param name="vOut">Output Double array.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="value">Constant value.</param>
                static void AddC64f(IntPtr vIn, IntPtr vOut, k64s count, k64f value)
                {
                    KCheck(kMath_AddC64f((k64f*)vIn.ToPointer(), (k64f*)vOut.ToPointer(), (kSize)count, value));
                }

                /// <summary>Subtracts a constant from each element in an input array and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn">Input Int32 array.</param>
                /// <param name="vOut">Output Int32 array.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="value">Constant value.</param>
                static void SubC32s(IntPtr vIn, IntPtr vOut, k64s count, k32s value)
                {
                    KCheck(kMath_SubC32s((k32s*)vIn.ToPointer(), (k32s*)vOut.ToPointer(), (kSize)count, value));
                }

                /// <summary>Subtracts a constant from each element in an input array and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn">Input Double array.</param>
                /// <param name="vOut">Output Double array.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="value">Constant value.</param>
                static void SubC64f(IntPtr vIn, IntPtr vOut, k64s count, k64f value)
                {
                    KCheck(kMath_SubC64f((k64f*)vIn.ToPointer(), (k64f*)vOut.ToPointer(), (kSize)count, value));
                }

                /// <summary>Multiplies each element in an input array by a constant and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn">Input Int32 array.</param>
                /// <param name="vOut">Output Int32 array.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="value">Constant value.</param>
                static void MulC32s(IntPtr vIn, IntPtr vOut, k64s count, k32s value)
                {
                    KCheck(kMath_MulC32s((k32s*)vIn.ToPointer(), (k32s*)vOut.ToPointer(), (kSize)count, value));
                }

                /// <summary>Multiplies each element in an input array by a constant and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn">Input Double array.</param>
                /// <param name="vOut">Output Double array.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="value">Constant value.</param>
                static void MulC64f(IntPtr vIn, IntPtr vOut, k64s count, k64f value)
                {
                    KCheck(kMath_MulC64f((k64f*)vIn.ToPointer(), (k64f*)vOut.ToPointer(), (kSize)count, value));
                }

                /// <summary>Divides each element in an input array by a constant and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn">Input Int32 array.</param>
                /// <param name="vOut">Output Int32 array.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="value">Constant value.</param>
                static void DivC32s(IntPtr vIn, IntPtr vOut, k64s count, k32s value)
                {
                    KCheck(kMath_DivC32s((k32s*)vIn.ToPointer(), (k32s*)vOut.ToPointer(), (kSize)count, value));
                }

                /// <summary>Divides each element in an input array by a constant and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn">Input Double array.</param>
                /// <param name="vOut">Output Double array.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="value">Constant value.</param>
                static void DivC64f(IntPtr vIn, IntPtr vOut, k64s count, k64f value)
                {
                    KCheck(kMath_DivC64f((k64f*)vIn.ToPointer(), (k64f*)vOut.ToPointer(), (kSize)count, value));
                }

                /// <summary>Limits each element in an input array using a minimum and maximum value.</summary>
                /// 
                /// <param name="vIn">Input Int32 array.</param>
                /// <param name="vOut">Output Int32 array.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="minValue">Minimum value.</param>
                /// <param name="maxValue">Maximum value.</param>
                static void ClampC32s(IntPtr vIn, IntPtr vOut, k64s count, k32s minValue, k32s maxValue)
                {
                    KCheck(kMath_ClampC32s((k32s*)vIn.ToPointer(), (k32s*)vOut.ToPointer(), (kSize)count, minValue, maxValue));
                }

                /// <summary>Limits each element in an input array using a minimum and maximum value.</summary>
                /// 
                /// <param name="vIn">Input Double array.</param>
                /// <param name="vOut">Output Double array.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="minValue">Minimum value.</param>
                /// <param name="maxValue">Maximum value.</param>
                static void ClampC64f(IntPtr vIn, IntPtr vOut, k64s count, k64f minValue, k64f maxValue)
                {
                    KCheck(kMath_ClampC64f((k64f*)vIn.ToPointer(), (k64f*)vOut.ToPointer(), (kSize)count, minValue, maxValue));
                }

                /// <summary>Compares each element in an input array with a specified value, and replaces all matching values with another given value.</summary>
                /// 
                /// <param name="vIn">Input Int32 array.</param>
                /// <param name="vOut">Output Int32 array.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="comparison">Comparator used to determine a match.</param>
                /// <param name="value">Value for comparison.</param>
                /// <param name="replacement"> Value for replacement.</param>
                static void ReplaceC32s(IntPtr vIn, IntPtr vOut, k64s count, KComparison comparison, k32s value, k32s replacement)
                {
                    KCheck(kMath_ReplaceC32s((k32s*)vIn.ToPointer(), (k32s*)vOut.ToPointer(), (kSize)count, comparison, value, replacement));
                }

                /// <summary>Compares each element in an input array with a specified value, and replaces all matching values with another given value.</summary>
                /// 
                /// <param name="vIn">Input Double array.</param>
                /// <param name="vOut">Output Double array.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="comparison">Comparator used to determine a match.</param>
                /// <param name="value">Value for comparison.</param>
                /// <param name="replacement">Value for replacement.</param>
                static void ReplaceC64f(IntPtr vIn, IntPtr vOut, k64s count, KComparison comparison, k64f value, k64f replacement)
                {
                    KCheck(kMath_ReplaceC64f((k64f*)vIn.ToPointer(), (k64f*)vOut.ToPointer(), (kSize)count, comparison, value, replacement));
                }

                /// <summary>Compares each element in an input array with a specified value and stores the results in an output array.</summary>
                /// 
                /// <param name="vIn">Input Int32array.</param>
                /// <param name="vOut">Output Int32 array (0 or 1, depending on comparison result).</param>
                /// <param name="count">Count of values.</param>
                /// <param name="comparison">Comparator used to determine a match.</param>
                /// <param name="value">Value for comparison.</param>
                static void CompareC32s(IntPtr vIn, IntPtr vOut, k64s count, KComparison comparison, k32s value)
                {
                    KCheck(kMath_CompareC32s((k32s*)vIn.ToPointer(), (k32s*)vOut.ToPointer(), (kSize)count, comparison, value));
                }

                /// <summary>Compares each element in an input array with a specified value and stores the results in an output array.</summary>
                /// 
                /// <param name="vIn">Input Double array.</param>
                /// <param name="vOut">Output Int32 array (0 or 1, depending on comparison result).</param>
                /// <param name="count">Count of values.</param>
                /// <param name="comparison">Comparator used to determine a match.</param>
                /// <param name="value">Value for comparison.</param>
                static void CompareC64f(IntPtr vIn, IntPtr vOut, k64s count, KComparison comparison, k64f value)
                {
                    KCheck(kMath_CompareC64f((k64f*)vIn.ToPointer(), (k32s*)vOut.ToPointer(), (kSize)count, comparison, value));
                }

                /// <summary>Adds the values in two input arrays and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn1">First Int32 input array.</param>
                /// <param name="vIn2">Second Int32 input array.</param>
                /// <param name="vOut">Output Double array.</param>
                /// <param name="count">Count of values.</param>
                static void Add32s(IntPtr vIn1, IntPtr vIn2, IntPtr vOut, k64s count)
                {
                    KCheck(kMath_Add32s((k32s*)vIn1.ToPointer(), (k32s*)vIn2.ToPointer(), (k32s*)vOut.ToPointer(), (kSize)count));
                }

                /// <summary>Adds the values in two input arrays and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn1">First Double input array.</param>
                /// <param name="vIn2">Second Double input array.</param>
                /// <param name="vOut">Output Double array.</param>
                /// <param name="count">Count of values.</param>
                static void Add64f(IntPtr vIn1, IntPtr vIn2, IntPtr vOut, k64s count)
                {
                    KCheck(kMath_Add64f((k64f*)vIn1.ToPointer(), (k64f*)vIn2.ToPointer(), (k64f*)vOut.ToPointer(), (kSize)count));
                }

                /// <summary>Subtracts the values in two input arrays and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn1">First Int32 input array.</param>
                /// <param name="vIn2">Second Int32 input array.</param>
                /// <param name="vOut">Output Double array.</param>
                /// <param name="count">Count of values.</param>
                static void Sub32s(IntPtr vIn1, IntPtr vIn2, IntPtr vOut, k64s count)
                {
                    KCheck(kMath_Sub32s((k32s*)vIn1.ToPointer(), (k32s*)vIn2.ToPointer(), (k32s*)vOut.ToPointer(), (kSize)count));
                }

                /// <summary>Subtracts the values in two input arrays and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn1">First Double input array.</param>
                /// <param name="vIn2">Second Double input array.</param>
                /// <param name="vOut">Output Double array.</param>
                /// <param name="count">Count of values.</param>
                static void Sub64f(IntPtr vIn1, IntPtr vIn2, IntPtr vOut, k64s count)
                {
                    KCheck(kMath_Sub64f((k64f*)vIn1.ToPointer(), (k64f*)vIn2.ToPointer(), (k64f*)vOut.ToPointer(), (kSize)count));
                }

                /// <summary>Multiplies the values in two input arrays and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn1">First Int32 input array.</param>
                /// <param name="vIn2">Second Int32 input array.</param>
                /// <param name="vOut">Output Double array.</param>
                /// <param name="count">Count of values.</param>
                static void Mul32s(IntPtr vIn1, IntPtr vIn2, IntPtr vOut, k64s count)
                {
                    KCheck(kMath_Mul32s((k32s*)vIn1.ToPointer(), (k32s*)vIn2.ToPointer(), (k32s*)vOut.ToPointer(), (kSize)count));
                }

                /// <summary>Multiplies the values in two input arrays and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn1">First Double input array.</param>
                /// <param name="vIn2">Second Double input array.</param>
                /// <param name="vOut">Output Double array.</param>
                /// <param name="count">Count of values.</param>
                static void Mul64f(IntPtr vIn1, IntPtr vIn2, IntPtr vOut, k64s count)
                {
                    KCheck(kMath_Mul64f((k64f*)vIn1.ToPointer(), (k64f*)vIn2.ToPointer(), (k64f*)vOut.ToPointer(), (kSize)count));
                }

                /// <summary>Divides the values in two input arrays and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn1">First Int32 input array.</param>
                /// <param name="vIn2">Second Int32 input array.</param>
                /// <param name="vOut">Output Double array.</param>
                /// <param name="count">Count of values.</param>
                static void Div32s(IntPtr vIn1, IntPtr vIn2, IntPtr vOut, k64s count)
                {
                    KCheck(kMath_Div32s((k32s*)vIn1.ToPointer(), (k32s*)vIn2.ToPointer(), (k32s*)vOut.ToPointer(), (kSize)count));
                }

                /// <summary>Divides the values in two input arrays and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn1">First Double input array.</param>
                /// <param name="vIn2">Second Double input array.</param>
                /// <param name="vOut">Output Double array.</param>
                /// <param name="count">Count of values.</param>
                static void Div64f(IntPtr vIn1, IntPtr vIn2, IntPtr vOut, k64s count)
                {
                    KCheck(kMath_Div64f((k64f*)vIn1.ToPointer(), (k64f*)vIn2.ToPointer(), (k64f*)vOut.ToPointer(), (kSize)count));
                }

                /// <summary>Calculates the moving average over an input array and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn">Input Int32 array.</param>
                /// <param name="vOut">Second Int32 input array.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="window">Moving average window size</param>
                static void MovingAvg32s(IntPtr vIn, IntPtr vOut, k64s count, k64s window)
                {
                    KCheck(kMath_MovingAvg32s((k32s*)vIn.ToPointer(), (k32s*)vOut.ToPointer(), (kSize)count, (kSize)window));
                }

                /// <summary>Calculates the moving average over an input array and stores the result in an output array.</summary>
                /// 
                /// <param name="input">Input Int32 List.</param>
                /// <param name="window">Moving average window size</param>
                /// <returns>A List containing the averages.</returns>
                static array<int>^ MovingAvg32s(array<int>^ input, int window)
                {
                    pin_ptr<k32s> pinIn = &(input[0]);
                    k32s* vIn = (k32s*)pinIn;

                    int count = input->Length - window + 1;
                    KArray1^ output = gcnew KArray1(K32s::KTypeId, count);
                    k32s* vOut = (k32s*)(output->Data.ToPointer());

                    KCheck(kMath_MovingAvg32s(vIn, vOut, input->Length, window));

                    array<k32s>^ arrOut = gcnew array<k32s>(count);
                    Marshal::Copy(output->Data, arrOut, 0, count);

                    return arrOut;
                }

                /// <summary>Calculates the moving average over an input array and stores the result in an output array.</summary>
                /// 
                /// <param name="vIn">Input Double array.</param>
                /// <param name="vOut">Second Double input array.</param>
                /// <param name="count">Count of values.</param>
                /// <param name="window">Moving average window size</param>
                static void MovingAvg64f(IntPtr vIn, IntPtr vOut, k64s count, k64s window)
                {
                    KCheck(kMath_MovingAvg64f((k64f*)vIn.ToPointer(), (k64f*)vOut.ToPointer(), (kSize)count, (kSize)window));
                }
                
                /// <summary>Calculates the moving average over an input array and stores the result in an output array.</summary>
                /// 
                /// <param name="input">Input Double List.</param>
                /// <param name="window">Moving average window size</param>
                /// <returns>A List containing the averages.</returns>
                static array<double>^ MovingAvg64f(array<double>^ input, int window)
                {
                    pin_ptr<k64f> pinIn = &(input[0]);
                    k64f* vIn = (k64f*)pinIn;

                    int count = input->Length - window + 1;
                    KArray1^ output = gcnew KArray1(K64f::KTypeId, count);
                    k64f* vOut = (k64f*)(output->Data.ToPointer());

                    KCheck(kMath_MovingAvg64f(vIn, vOut, input->Length, window));

                    array<k64f>^ arrOut = gcnew array<k64f>(count);
                    Marshal::Copy(output->Data, arrOut, 0, count);

                    return arrOut;
                }

                /// <summary>Returns the greatest common divisor of two integers.</summary>
                /// 
                /// <param name="a">First integer.</param>
                /// <param name="b">Second integer.</param>
                /// <returns>Result.</returns>
                static k32s Gcd32s(k32s a, k32s b)
                {
                    k32s result; 

                    kMath_Gcd32s(a, b, &result); 

                    return result; 
                }

                /// <summary>Returns the least common multiple of two integers.</summary>
                /// 
                /// <param name="a">First integer.</param>
                /// <param name="b">Second integer.</param>
                /// <returns>Result.</returns>
                static k32s Lcm32s(k32s a, k32s b)
                {
                    k32s result;

                    kMath_Lcm32s(a, b, &result);

                    return result;
                }

                /// <summary>Calculates the base-2 logarithm of the input, rounded up to the nearest integer.</summary>
                /// 
                /// <param name="a">Input value (must be positive).</param>
                /// <returns>Base-2 logarithm of input.</returns>
                static k32s Log2Ceil32s(k32s a)
                {
                    return kMath_Log2Ceil32u((k32u)a); 
                }

                /// <summary>Rounds the input up to the nearest integer.</summary>
                /// 
                /// <param name="a">Input value.</param>
                /// <returns>Rounded value.</returns>
                static k64f Round64f(k64f a)
                {
                    return kMath_Round64f(a); 
                }

            private:
                KMath() : KObject(DefaultRefStyle) {}
            };
        }
    }
}

#endif
