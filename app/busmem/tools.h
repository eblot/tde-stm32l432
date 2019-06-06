/**
 * Tools and miscelleanous helpers
 */

#ifndef _TOOLS_H_
#define _TOOLS_H_

#include <stdint.h>

//-----------------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------------

#ifndef ASSERT_COMPILE
/**
 * Compile-time assertion.
 * Aborts the compilation if the c' constant expression expression evaluates
 * to false
 */
#define _JOIN_(_x_,_y_) _DO_JOIN_(_x_,_y_)
#define _DO_JOIN_(_x_,_y_) _x_##_y_
#define ASSERT_COMPILE(c) \
  void _JOIN_(assert_compile, __LINE__)(int assert_compile[(c)?1:-1])
#endif // ASSERT_COMPILE

/** Compute the size of an array */
#define ARRAY_SIZE(_a_) (sizeof(_a_)/sizeof(_a_[0]))
/** Compute the length of a zero-terminated string */
#define ZARRAY_SIZE(_a_) (ARRAY_SIZE(_a_)-1)
/** Compute the minimum value */
#define MIN(_a_, _b_) ((_a_)<(_b_) ? (_a_):(_b_))
/** Compute the maximum value */
#define MAX(_a_, _b_) ((_a_)>(_b_) ? (_a_):(_b_))
/** Compute the absolute value */
#define ABS(_a_) ((_a_)>0 ? (_a_):(-(_a_)))

/** stringification macro, step 1 */
#define _XSTR_(s) _STR_(s)
/** stringification macro, step 2 */
#define _STR_(s) #s

#ifdef APP_NAME
/** GIT external reference step 3 */
# define __GIT_REFERENCE(_app_,_type_) git_##_app_##_build_##_type_
/** GIT external reference step 3 */
# define _GIT_REFERENCE(_app_,_type_) __GIT_REFERENCE(_app_, _type_)
/** GIT external reference step 1 */
# define GIT_REFERENCE(_type_) _GIT_REFERENCE(APP_NAME, _type_)
#endif // APP_NAME

#ifdef LED_DEBUG
# define LED_CODE(_led_, _code_) led_code(_led_, _code_)
#else // LED_DEBUG
# define LED_CODE(_led_, _code_)
#endif // !LED_DEBUG

#ifndef ASSERT_COMPILE
#define _JOIN_(_x_,_y_) _DO_JOIN_(_x_,_y_)
#define _DO_JOIN_(_x_,_y_) _x_##_y_
#define ASSERT_COMPILE(_c_) \
  void _JOIN_(assert_compile, __LINE__)(int assert_compile[(_c_)?1:-1])
#endif // ASSERT_COMPILE

/** Compute the size of an array */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(_a_) (sizeof(_a_)/sizeof(_a_[0]))
#endif // ARRAY_SIZE

#ifndef _STRUCT_MEMBER
/** Structure member of a type */
#define _STRUCT_MEMBER(_type_, _mbr_) (((_type_ *)(NULL))->_mbr_)
#endif // SIZEOF

#ifndef STRUCT_TYPEOF
/** Type of a structure member */
#define STRUCT_TYPEOF(_type_, _mbr_) typeof(_STRUCT_MEMBER(_type_, _mbr_))
#endif // SIZEOF

#ifndef SIZEOF_MEMBER
/** Compute the size of a structure member */
#define SIZEOF_MEMBER(_type_, _mbr_) sizeof(_STRUCT_MEMBER(_type_, _mbr_))
#endif // SIZEOF_MEMBER

#ifndef ARRAY_SIZEOF_MEMBER
/** Compute the size of a structure member */
#define ARRAY_SIZEOF_MEMBER(_type_, _mbr_) \
   (sizeof(((_type_ *)(NULL))->_mbr_)/sizeof(((_type_ *)(NULL))->_mbr_[0]))
#endif // ARRAY_SIZEOF_MEMBER

#ifdef __GNUC__
#define _unused __attribute__((__unused__))
#else
#define _unused
#endif //

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------

#define ENOERR    0    /**< No error, not defined in errno.h */

/** Debug leds */
enum led {
    led_green, /**< Green led */
    led_red,   /**< Red led */
};

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

void led_code(enum led led, unsigned int code);

//-----------------------------------------------------------------------------
// Bytes and bits inline function helpers
//-----------------------------------------------------------------------------

/** Force alignement on a 16-bit boundary */
#define ALIGN_UINT16 __attribute__ ((aligned(sizeof(uint16_t))))
/** Force alignement on a 32-bit boundary */
#define ALIGN_UINT32 __attribute__ ((aligned(sizeof(uint32_t))))
/** Force alignement on a 64-bit boundary */
#define ALIGN_UINT64 __attribute__ ((aligned(sizeof(uint64_t))))


/**
 * Read a 8-bit integer from a byte stream, using host (little) endianess
 *
 * @param[out] value updated with the 8-bit integer value
 * @param[in] buf the input byte stream
 */
static inline void
get_uint8(uint8_t * value, const uint8_t * buf)
{
    *value = ((uint8_t)buf[0]);
}

/**
 * Read a 16-bit integer from a byte stream, using host (little) endianess
 *
 * @param[out] value updated with the 16-bit integer value
 * @param[in] buf the input byte stream
 */
static inline void
get_uint16(uint16_t * value, const uint8_t * buf)
{
    *value = (uint16_t)(((unsigned int)buf[0]) |
                        (((unsigned int)buf[1])<<8));
}

/**
 * Read a 32-bit integer from a byte stream, using host (little) endianess
 *
 * @param[out] value updated with the 32-bit integer value
 * @param[in] buf the input byte stream
 */
static inline void
get_uint32(uint32_t * value, const uint8_t * buf)
{
    *value = (uint32_t)(((unsigned int)buf[0])       |
                        (((unsigned int)buf[1])<<8)  |
                        (((uint32_t)buf[2])<<16) |
                        (((uint32_t)buf[3])<<24));
}

/**
 * Append a 8-bit integer to a byte stream, using host (little) endianess
 *
 * @param[out] buf the byte stream to receive the integer value
 * @param[in] value the 8-bit integer value to insert
 */
static inline void
set_uint8(uint8_t * buf, uint8_t value)
{
    buf[0] = value;
}

/**
 * Append a 16-bit integer to a byte stream, using host (little) endianess
 *
 * @param[out] buf the byte stream to receive the integer value
 * @param[in] value the 16-bit integer value to insert
 */
static inline void
set_uint16(uint8_t * buf, uint16_t value)
{
    buf[0] = (uint8_t)value; buf[1] = (uint8_t)(value>>8);
}

/**
 * Append a 32-bit integer to a byte stream, using host (little) endianess
 *
 * @param[out] buf the byte stream to receive the integer value
 * @param[in] value the 32-bit integer value to insert
 */
static inline void
set_uint32(uint8_t * buf, uint32_t value)
{
    buf[0] = (uint8_t)value;
    buf[1] = (uint8_t)(value>>8);
    buf[2] = (uint8_t)(value>>16);
    buf[3] = (uint8_t)(value>>24);
}

/**
 * Reverse bits in a 16-bit word (LSB <-> MSB style).
 * Bit reversal is done by interchanging adjacent single bits, then
 * interchanging adjacent 2-bit fields, and so on (nibble, byte)
 *
 * @param[in] value input half-word to reverse
 * @return the reversed half-word
 */
static inline uint16_t
bit_swap16(uint16_t value)
{
    value = (uint16_t)((value & 0x5555U) << 1) | ((value >> 1) & 0x5555U);
    value = (uint16_t)((value & 0x3333U) << 2) | ((value >> 2) & 0x3333U);
    value = (uint16_t)((value & 0x0f0fU) << 4) | ((value >> 4) & 0x0f0fU);
    value = (uint16_t)((value & 0x00ffU) << 8) | ((value >> 8) & 0x00ffU);

    return value;
}

/**
 * Swap bytes in a 16-bit word (ab-> ba style).
 *
 * @param[in] value input word to swap
 * @return the swapped word
 */
static inline uint16_t
byte_swap16(uint16_t value)
{
   return (uint16_t)((value >> 8U) | (value << 8U));
}

/**
 * Swap bytes in a 32-bit word (abcd -> dcba style).
 *
 * @param[in] value input word to swap
 * @return the swapped word
 */
static inline uint32_t
byte_swap32(uint32_t value)
{
   return (((uint32_t)byte_swap16((uint16_t)value))<<16U) |
          (byte_swap16((uint16_t)(value>>16U)));
}

/**
 * Count the number of '1' bit in a native word
 * @note efficient implementation with no loop and usual bit hacking
 *
 * @param[in] x the word to count the number of '1' bits
 * @return the count of bits set in the word
 */
static inline unsigned int
bit_ones(unsigned int x)
{
   x = (x & 0x5555U) + ((x >> 1 ) & 0x5555U);
   x = (x & 0x3333U) + ((x >> 2 ) & 0x3333U);
   x = (x & 0x0F0FU) + ((x >> 4 ) & 0x0F0FU);
   x = (x & 0x00FFU) + ((x >> 8 ) & 0x00FFU);
   return 16U - x;
}

/**
 * CLZ: count leading zeroes
 * @note CLZ builtin does not exist with TI and is SW-coded in GCC
 * @note CLZ is not defined for a zeroed word
 *
 * @param[in] x the word to count the number leading zeros
 * @return CLZ
 */
static inline unsigned int
bit_clz(uint16_t x)
{
   x |= (x >> 1);
   x |= (x >> 2);
   x |= (x >> 4);
   x |= (x >> 8);
   return bit_ones(x);
}

#endif // _TOOLS_H_

