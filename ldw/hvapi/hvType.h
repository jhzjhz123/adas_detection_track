#ifndef __HVTYPE_H__
#define __HVTYPE_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef _WIN32
#	define HV_API __cdecl
#	define HV_CBI __stdcall
#else
#	define HV_API
#	define HV_CBI
#endif //_WIN32

/** HV_IN is used to identify inputs to an HV function.  This designation
    will also be used in the case of a pointer that points to a parameter
    that is used as an output. */
#ifndef HV_IN
#define HV_IN
#endif

/** HV_OUT is used to identify outputs from an HV function.  This
    designation will also be used in the case of a pointer that points
    to a parameter that is used as an input. */
#ifndef HV_OUT
#define HV_OUT
#endif

/** HV_INOUT is used to identify parameters that may be either inputs or
    outputs from an HV function at the same time.  This designation will
    also be used in the case of a pointer that  points to a parameter that
    is used both as an input and an output. */
#ifndef HV_INOUT
#define HV_INOUT
#endif

#define HV_MAX_ENUM_VALUE	0X7FFFFFFF

/** HV_HVID */
typedef void HV_HVID;

/** HV_U8 is an 8 bit unsigned quantity that is byte aligned */
typedef unsigned char HV_U8;

/** HV_BYTE is an 8 bit unsigned quantity that is byte aligned */
typedef unsigned char HV_BYTE;

/** HV_S8 is an 8 bit signed quantity that is byte aligned */
typedef signed char HV_S8;

/** HV_CHAR is an 8 bit signed quantity that is byte aligned */
typedef char HV_CHAR;

/** HV_U16 is a 16 bit unsigned quantity that is 16 bit word aligned */
typedef unsigned short HV_U16;

/** HV_WCHAR is a 16 bit unsigned quantity that is 16 bit word aligned */
#if defined _WIN32
typedef unsigned short HV_WCHAR;
typedef unsigned short* HV_PWCHAR;
#elif defined LINUX
typedef unsigned char HV_WCHAR;
typedef unsigned char* HV_PWCHAR;
#endif

/** HV_S16 is a 16 bit signed quantity that is 16 bit word aligned */
typedef signed short HV_S16;

/** HV_U32 is a 32 bit unsigned quantity that is 32 bit word aligned */
typedef unsigned int HV_U32;

/** HV_S32 is a 32 bit signed quantity that is 32 bit word aligned */
typedef signed int HV_S32;

/*HV_F32 is a 32 bit float*/
typedef float HV_F32;

/* Users with compilers that cannot accept the "long long" designation should
   define the HV_SKIP64BIT macro.  It should be noted that this may cause
   some components to fail to compile if the component was written to require
   64 bit integral types.  However, these components would NOT compile anyway
   since the compiler does not support the way the component was written.
*/
#ifndef HV_SKIP64BIT
#ifdef _WIN32
/** HV_U64 is a 64 bit unsigned quantity that is 64 bit word aligned */
typedef unsigned __int64  HV_U64;
/** HV_S64 is a 64 bit signed quantity that is 64 bit word aligned */
typedef signed   __int64  HV_S64;
#else // WIN32
/** HV_U64 is a 64 bit unsigned quantity that is 64 bit word aligned */
typedef unsigned long long HV_U64;
/** HV_S64 is a 64 bit signed quantity that is 64 bit word aligned */
typedef signed long long HV_S64;
#endif // WIN32
#endif // HV_SKIP64BIT

/** The HV_BOOL type is intended to be used to represent a true or a false
    value when passing parameters to and from the HV core and components.  The
    HV_BOOL is a 32 bit quantity and is aligned on a 32 bit word boundary.
 */
typedef enum HV_BOOL {
    HV_FALSE = 0,
    HV_TRUE = !HV_FALSE,
	HV_BOOL_MAX = HV_MAX_ENUM_VALUE
} HV_BOOL;

/** The HV_PTR type is intended to be used to pass pointers between the HV
    applications and the HV Core and components.  This is a 32 bit pointer and
    is aligned on a 32 bit boundary.
 */
typedef void* HV_PTR;

/** The HV_HANDLE type is intended to be used to pass pointers between the HV
    applications and the HV Core and components.  This is a 32 bit pointer and
    is aligned on a 32 bit boundary.
 */
typedef void* HV_HANDLE;

/** The HV_STRING type is intended to be used to pass "C" type strings between
    the application and the core and component.  The HV_STRING type is a 32
    bit pointer to a zero terminated string.  The  pointer is word aligned and
    the string is byte aligned.
 */
typedef char* HV_PCHAR;

/** The HV_PBYTE type is intended to be used to pass arrays of bytes such as
    buffers between the application and the component and core.  The HV_PBYTE
    type is a 32 bit pointer to a zero terminated string.  The  pointer is word
    aligned and the string is byte aligned.
 */
typedef unsigned char* HV_PBYTE;

/** The HV_PTCHAR type is intended to be used to pass arrays of wchar such as
    unicode char between the application and the component and core.  The HV_PTCHAR
    type is a 32 bit pointer to a zero terminated string.  The  pointer is word
    aligned and the string is byte aligned.
 */

#ifdef _UNICODE
typedef unsigned short* HV_PTTCHAR;
typedef unsigned short HV_TTCHAR;
#else
typedef char* HV_PTTCHAR;
typedef char HV_TTCHAR;
#endif


#ifndef NULL
#ifdef __cplusplus
#define NULL    0
#else
#define NULL    ((void *)0)
#endif
#endif

/**
 * Input stream format, Frame or Stream..
 */
typedef enum {
    HV_INPUT_FRAME	= 1,	/*!< Input contains completely frame(s) data. */
    HV_INPUT_STREAM,		/*!< Input is stream data. */
	HV_INPUT_STREAM_MAX = HV_MAX_ENUM_VALUE
} HV_INPUT_TYPE;


/**
 * General data buffer, used as input or output.
 */
typedef struct {
	HV_PBYTE    Buffer;		/*!< Buffer pointer */
	HV_U32      Length;		/*!< Buffer size in byte */
	HV_S64      Time;		/*!< The time of the buffer */
	HV_S32		Width;		/*frame width*/
	HV_S32		Height;		/*frame height*/
} HV_CODECBUFFER;


/**
 * GUID structure...
 */
typedef struct _HV_GUID {
	HV_U32	Data1;		/*!< Data1 value */
	HV_U16	Data2;		/*!< Data2 value */
	HV_U16	Data3;		/*!< Data3 value */
	HV_U8	Data4[8];	/*!< Data4 value */
} HV_GUID;


/**
 * Head Info description
 */
typedef struct _HV_HEAD_INFO {
	HV_PBYTE	Buffer;				/*!< [In] Head Buffer pointer */
	HV_U32		Length;				/*!< [In] Head Buffer size in byte */
	HV_PCHAR	Description;		/*!< [In/Out] Allocated by Caller. The char buffer of description  */
	HV_U32		Size;				/*!< [In] The size of description  */
} HV_HEAD_INFO;


/**
 * The init memdata flag.
 */
typedef enum{
	HV_IMF_USERMEMOPERATOR		=0,	/*!< memData is  the pointer of memoperator function*/
	HV_IMF_PREALLOCATEDBUFFER	=1,	/*!< memData is  preallocated memory*/
	HV_IMF_MAX = HV_MAX_ENUM_VALUE
}HV_INIT_MEM_FlAG;


typedef struct
{
	HV_PTR	pUserData;
	HV_PTR	(HV_API * LoadLib) (HV_PTR pUserData, HV_PCHAR pLibName, HV_S32 nFlag);
	HV_PTR	(HV_API * GetAddress) (HV_PTR pUserData, HV_PTR hLib, HV_PCHAR pFuncName, HV_S32 nFlag);
	HV_S32	(HV_API * FreeLib) (HV_PTR pUserData, HV_PTR hLib, HV_S32 nFlag);
} HV_LIB_OPERATOR;

/**
 * The init memory structure..
 */
typedef struct{
	HV_U32						memflag;		/*!<memory and other param flag  */
	HV_PTR						memData;		/*!<a pointer to HV_MEM_OPERATOR or a preallocated buffer  */
	HV_LIB_OPERATOR *			libOperator;	/*!<Library operator function pointer. If memflag is 0X1x, the param is available  */
	HV_U32						reserved1;		/*!<reserved  */
	HV_U32						reserved2;		/*!<reserved */
	HV_F32						camera_yaw; 
	HV_F32						camera_pitch;
	HV_F32						camera_roll;
	HV_F32						g_k1;
	HV_F32						g_k2;
	HV_F32						g_k3;
	HV_F32						g_p1;
	HV_F32						g_p2;
	HV_S32						g_fx;
	HV_S32						g_fy;
	HV_S32						g_cx;
	HV_S32						g_cy;
	HV_F32						g_camera_position;
	HV_F32						g_camera_height;
	HV_F32						g_camera_head;
	signed long(HV_API * hv_clock) ();
	int(HV_API * hv_printf) (const char *format, ...);
	int(HV_API *hv_rand) ();
}HV_INIT_USERDATA;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // __voType_H__
