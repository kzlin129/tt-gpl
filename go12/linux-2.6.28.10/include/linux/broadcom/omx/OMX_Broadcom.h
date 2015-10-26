/*****************************************************************************
* Copyright 2001 - 2008 Broadcom Corporation.  All rights reserved.
*
* Unless you and Broadcom execute a separate written software license
* agreement governing use of this software, this software is licensed to you
* under the terms of the GNU General Public License version 2, available at
* http://www.broadcom.com/licenses/GPLv2.php (the "GPL"). 
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a
* license other than the GPL, without Broadcom's express prior written
* consent.
*
*****************************************************************************/

#ifndef OMX_Broadcom_h
#define OMX_Broadcom_h

#include "OMX_Component.h"

#define __ilpacked__ __attribute__ ((packed)) 

// for use in buffer headers - marks the contained data
// as being a codec header
#define OMX_BUFFERFLAG_TIME_UNKNOWN 0x00000100

// types here are used with the vendor specific index tags used:
//      used for ilcs-top communication
//      OMX_IndexParamMarkComparison, /**< reference: OMX_PARAM_MARKCOMPARISONTYPE */
//      OMX_IndexParamPortSummary, /**< reference: OMX_PARAM_PORTSUMMARYTYPE */
//      OMX_IndexParamTunnelStatus, /**< reference : OMX_PARAM_TUNNELSTATUSTYPE */
//
//      used for top-ril communication
//      OMX_IndexParamBufferAddress, /**< reference : OMX_PARAM_BUFFERADDRESSTYPE */
//      OMX_IndexParamTunnelSetup, /**< reference : OMX_PARAM_TUNNELSETUPTYPE */
//
//      used for ril-ril communication
//      OMX_IndexParamImagePoolDisplayFunction, /**<reference : OMX_PARAM_IMAGEPOOLDISPLAYFUNCTIONTYPE */
//      OMX_IndexConfigAudioHandle,/**<reference : OMX_CONFIG_AUDIOHANDLE */
//      OMX_IndexParamBrcmDataUnitType, /**<reference: OMX_PARAM_DATAUNITTYPE */
//      OMX_IndexParamCodecConfig, /**<reference: OMX_PARAM_CODECCONFIGTYPE */
//      OMX_IndexParamCameraPoolToEncoderFunction, /**<reference : OMX_PARAM_CAMERAPOOLTOENCODERFUNCTIONTYPE */
//      OMX_IndexParamCameraStripeFunction,     /**<reference : OMX_PARAM_CAMERASTRIPEFUNCTIONTYPE */
//
//      used for client-ril communication
//      OMX_IndexParamTestInterface, /**< reference : OMX_PARAM_TESTINTERFACETYPE */
//      OMX_IndexConfigDisplayRegion, /**< reference : OMX_CONFIG_DISPLAYREGIONTYPE */
//      OMX_IndexParamSource, /**< reference : OMX_PARAM_SOURCETYPE */
//      OMX_IndexParamSourceSeed, /**< reference : OMX_PARAM_SOURCESEEDTYPE */
//      OMX_IndexParamResize, /**< reference : OMX_PARAM_RESIZETYPE */
//      OMX_IndexConfigVisualisation, /**< reference : OMX_CONFIG_VISUALISATIONTYPE */
//      OMX_IndexConfigSingleStep, /**<reference : OMX_CONFIG_BOOLEANTYPE */
//      OMX_IndexParamAudioChannel, /**<reference: OMX_PARAM_AUDIOCHANNELTYPE */
//      OMX_IndexConfigPlayMode, /**<reference: OMX_CONFIG_PLAYMODETYPE */
//      OMX_IndexParamCameraCamplusId, /**<reference : OMX_PARAM_U32TYPE */
//      OMX_IndexParamCameraCdiId, /**<reference : OMX_PARAM_U32TYPE */
//      OMX_IndexParamCameraMode, /**<reference : OMX_PARAM_CAMERAMODETYPE */
//      OMX_IndexConfigAudioRecordCapture, /**<reference : OMX_CONFIG_BOOLEANTYPE */
//      OMX_IndexConfigCommonImageFilterParameters,  /**<reference : OMX_CONFIG_IMAGEFILTERPARAMSTYPE */
//      OMX_IndexConfigTransitionControl, /**<reference: OMX_CONFIG_TRANSITIONCONTROLTYPE */
//      OMX_IndexConfigPresentationOffset, /**<reference: OMX_TIME_CONFIG_TIMESTAMPTYPE */
//      OMX_IndexParamSourceFunctions,          /**<reference: OMX_PARAM_READSTILLFUNCTIONTYPE*/
//      OMX_IndexConfigVEReleaseCallback,       /**<reference: OMX_PARAM_VIDEOENCODERELEASEFUNCTIONTYPE*/
//      OMX_IndexConfigAudioMonoTrackControl,   /**<reference : OMX_CONFIG_AUDIOMONOTRACKCONTROLTYPE */
//      OMX_IndexParamCameraImagePool,          /**<reference : OMX_PARAM_CAMERAIMAGEPOOLTYPE */
//      OMX_IndexParamCameraISPOutputPoolHeight, /**<reference : OMX_PARAM_U32TYPE */
//      OMX_IndexParamImagePoolSize,            /**<reference: OMX_PARAM_IMAGEPOOLSIZETYPE */
//      OMX_IndexParamImagePoolExternal,        /**<reference: OMX_PARAM_IMAGEPOOLEXTERNALTYPE */
//      OMX_IndexParamRUTILFifoInfo,            /**<reference: OMX_PARAM_RUTILFIFOINFOTYPE*/
//      OMX_IndexParamILFifoConfig,             /**<reference: OMX_PARAM_ILFIFOCONFIG */
//      OMX_IndexConfigCommonSharpness,         /**<reference : OMX_CONFIG_SHARPNESSTYPE */
//      OMX_IndexConfigCommonFlickerCancellation,   /**reference : OMX_CONFIG_FLICKERCANCELTYPE */

//      OMX_IndexParamCameraSwapImagePools,     /**<reference : OMX_CONFIG_BOOLEANTYPE */
//      OMX_IndexParamCameraSingleBufferCaptureInput,  /**<reference : OMX_CONFIG_BOOLEANTYPE */
//      OMX_IndexConfigCommonRedEyeRemoval,   /**<reference : OMX_CONFIG_REDEYEREMOVALTYPE  */
//      OMX_IndexConfigCommonFaceDetectionControl,  /**<reference : OMX_CONFIG_FACEDECTECTIONCONTROLTYPE */
//      OMX_IndexConfigCommonFaceDetectionRegion,   /**<reference : OMX_CONFIG_FACEDETECTIONREGIONTYPE */
//      OMX_IndexConfigCommonInterlace,         /**<reference: OMX_COMMONINTERLACETYPE */

/* OMX_IndexParamBrcmDataUnit: Data Unit */
/*
This index uses the standard IL structure \code{OMX_PARAM_DATAUNITTYPE}.

The data unit is an indication to components connected to this
component of the type of data delivery available.
\code{OMX_DataUnitCodedPicture} indicates that we are able to give
framing information, using the \code{OMX_BUFFERFLAG_ENDOFFRAME} flag to
indicate that the data contained finishes a complete
frame. \code{OMX_DataUnitArbitraryStreamSection} indicates that no
end-of-frame markers will be present, and the decoder should perform
the steps necessary to decode the stream. The other enum values are
not used.
*/

/* OMX_IndexConfigPresentationOffset: Presentation Offset */
/*
This index uses the standard IL structure
\code{OMX_TIME_CONFIG_TIMESTAMPTYPE}.

The value of \code{nTimestamp} is added to the offset requested for
each new input frame. Takes effect for all new input frames, and has
no effect on the offset used for currently-queued frames. A positive
value will make the requested port earlier relative to other streams,
a negative value will make the requested port later relative to other
streams.
*/

/* OMX_IndexConfigSingleStep: Single Step */
/*
This index uses the standard IL structure \code{OMX_CONFIG_BOOLEANTYPE}.

When setting this config on a paused clock, where the \code{bEnabled}
value is OMX_TRUE, the media clock will advance to the next requested
media time. A paused clock is in running state but has a time scale of
0. This will trigger the display of the next video frame, so allowing
single-stepping functionality. This config can be set multiple times,
and will buffer up stepping requests until we have media requests to
fulfil, or the clock is stopped or un-paused.
*/

/* OMX_IndexParamCameraCamplusId: Camera Subsystem Identification */
/*
This index uses the standard IL structure \code{OMX_PARAM_U32TYPE}.

This parameter allows the configuration of the identifier to be used
to initialise the Broadcom Camplus subsystem that sits beneath the
camera component. If only one instance of the camera component is
used, the default value can be used. If more than one instance is
required, they must each have their own unique values for this
parameter. It is also used to tie the component to the image pool
created with \code{OMX_Set upCamPools}.
*/

typedef struct OMX_PARAM_MARKCOMPARISONTYPE {
   OMX_U32 nSize;            /**< Size of the structure in bytes */
   OMX_VERSIONTYPE nVersion; /**< OMX specification version information */
   OMX_PTR mark;             /**< Pointer to be used for mark comparisons */
} __ilpacked__ OMX_PARAM_MARKCOMPARISONTYPE;

typedef struct OMX_PARAM_PORTSUMMARYTYPE {
   OMX_U32 nSize;            /**< Size of the structure in bytes */
   OMX_VERSIONTYPE nVersion; /**< OMX specification version information */
   OMX_U32 nNumPorts;        /**< Total number of ports */
   OMX_U32 reqSet;           /**< Which set of ports is details below, portIndex[0] is port reqSet*32 */
   OMX_U32 portDir;          /**< Bitfield, 1 if output port, 0 if input port, max 256 ports */
   OMX_U32 portIndex[32];    /**< Port Indexes */
} __ilpacked__ OMX_PARAM_PORTSUMMARYTYPE;

typedef struct OMX_PARAM_TUNNELSTATUSTYPE {
   OMX_U32 nSize;            /**< Size of the structure in bytes */
   OMX_VERSIONTYPE nVersion; /**< OMX specification version information */
   OMX_U32 nPortIndex;       /**< Port being queried */
   OMX_PTR hTunneledComponent; /**< Component currently tunnelling with */
   OMX_U32 nTunneledPort;    /**< Port on tunnelled component */
} __ilpacked__ OMX_PARAM_TUNNELSTATUSTYPE;

/* OMX_IndexConfigDisplayRegion: Display Region */
typedef enum OMX_DISPLAYTRANSFORMTYPE{
   OMX_DISPLAY_ROT0 = 0,
   OMX_DISPLAY_MIRROR_ROT0 = 1,
   OMX_DISPLAY_MIRROR_ROT180 = 2,
   OMX_DISPLAY_ROT180 = 3,
   OMX_DISPLAY_MIRROR_ROT90 = 4,
   OMX_DISPLAY_ROT270 = 5,
   OMX_DISPLAY_ROT90 = 6,
   OMX_DISPLAY_MIRROR_ROT270 = 7,
   OMX_DISPLAY_DUMMY = 0x7FFFFFFF,
} OMX_DISPLAYTRANSFORMTYPE;

typedef struct OMX_DISPLAYRECTTYPE {
   OMX_S16 x_offset;
   OMX_S16 y_offset;
   OMX_S16 width;
   OMX_S16 height;
} __ilpacked__ OMX_DISPLAYRECTTYPE;

typedef enum OMX_DISPLAYMODETYPE {
   OMX_DISPLAY_MODE_FILL = 0,
   OMX_DISPLAY_MODE_LETTERBOX = 1,
   OMX_DISPLAY_MODE_DUMMY = 0x7FFFFFFF,
} OMX_DISPLAYMODETYPE;

typedef enum OMX_DISPLAYSETTYPE {
   OMX_DISPLAY_SET_NONE = 0,
   OMX_DISPLAY_SET_NUM = 1,
   OMX_DISPLAY_SET_FULLSCREEN = 2,
   OMX_DISPLAY_SET_TRANSFORM = 4,
   OMX_DISPLAY_SET_DEST_RECT = 8,
   OMX_DISPLAY_SET_SRC_RECT = 0x10,
   OMX_DISPLAY_SET_MODE = 0x20,
   OMX_DISPLAY_SET_PIXEL = 0x40,
   OMX_DISPLAY_SET_NOASPECT = 0x80,
   OMX_DISPLAY_SET_LAYER = 0x100,
   OMX_DISPLAY_SET_DUMMY = 0x7FFFFFFF,
} OMX_DISPLAYSETTYPE;

typedef struct OMX_CONFIG_DISPLAYREGIONTYPE {
   OMX_U32 nSize; 
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_DISPLAYSETTYPE set;
   OMX_U32 num;
   OMX_BOOL fullscreen;
   OMX_DISPLAYTRANSFORMTYPE transform;
   OMX_DISPLAYRECTTYPE dest_rect;
   OMX_DISPLAYRECTTYPE src_rect;
   OMX_BOOL noaspect;
   OMX_DISPLAYMODETYPE mode;
   OMX_U32 pixel_x;
   OMX_U32 pixel_y;
   OMX_S32 layer;
} __ilpacked__ OMX_CONFIG_DISPLAYREGIONTYPE;
/* 
This config sets the output display device, as well as the region used
on the output display, any display transformation, and some flags to
indicate how to scale the image.

The structure uses a bitfield, \code{set}, to indicate which fields are set
and should be used. All other fields will maintain their current
value.

\code{num} describes the display output device, with 0 typically being
a directly connected LCD display.

\code{fullscreen} indicates that we are using the full device screen
area, rather than a window of the display.  If fullscreen is false,
then dest_rect is used to specify a region of the display to use.

\code{transform} indicates any rotation or flipping used to map frames
onto the natural display orientation.

The \code{src_rect} indicates which area of the frame to display. If
all values are zero, the whole frame will be used.

The \code{noaspect} flag, if set, indicates that any display scaling
should disregard the aspect ratio of the frame region being displayed.

\code{mode} indicates how the image should be scaled to fit the
display. \code{OMX_DISPLAY_MODE_FILL} indicates that the image should
fill the screen by potentially cropping the frames.  Setting
\code{mode} to \code{OMX_DISPLAY_MODE_LETTERBOX} indicates that all
the source region should be displayed and black bars added if
necessary.

The \code{pixel_x} and \code{pixel_y} values, if non-zero, are used to
describe the size of a source pixel. If values are zero, then pixels
default to being square.

Set the \code{layer} that the image will appear on with the
\code{layer} field.
*/



/* OMX_IndexParamSource: Source Image Configuration */
typedef enum OMX_SOURCETYPE {
   OMX_SOURCE_WHITE = 0,
   OMX_SOURCE_BLACK = 1,
   OMX_SOURCE_DIAGONAL = 2,
   OMX_SOURCE_NOISE = 3,
   OMX_SOURCE_RANDOM = 4,
   OMX_SOURCE_COLOUR = 5,
   OMX_SOURCE_BLOCKS = 6,
   OMX_SOURCE_DUMMY = 0x7FFFFFFF,
} OMX_SOURCETYPE;

typedef struct OMX_PARAM_SOURCETYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_SOURCETYPE eType;
   OMX_U32 nParam;
   OMX_BOOL bSingleFrame;
} __ilpacked__ OMX_PARAM_SOURCETYPE;
/*
The source type determines the kind of image that is produced. Not all
combinations of source type and image type are supported. The
\code{nParam} field is used to specify the colour for the source
colour mode, and the offset of the diagonal pattern for diagonal mode.
The \code{bSingleFrame} parameter determines whether an EOS signal is
sent after a frame has been produced, and the component issues no more
data. If false, then no EOS signal is produced and data will
continuously be produced.
*/

/* OMX_IndexParamSourceSeed: Source Random Seed */
typedef struct OMX_PARAM_SOURCESEEDTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_U16 nData[16];
} __ilpacked__ OMX_PARAM_SOURCESEEDTYPE;
/*
This structure sets the current state of the random number generator
used for the random source type, allowing repeatable random image
creation.
*/

/* OMX_IndexParamResize: Resize Control */
typedef enum OMX_RESIZEMODETYPE {
   OMX_RESIZE_NONE,
   OMX_RESIZE_BOX,
   OMX_RESIZE_BYTES,
   OMX_RESIZE_DUMMY = 0x7FFFFFFF,
} OMX_RESIZEMODETYPE;

typedef struct OMX_PARAM_RESIZETYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_RESIZEMODETYPE eMode;
   OMX_U32 nMaxWidth;
   OMX_U32 nMaxHeight;
   OMX_U32 nMaxBytes;
   OMX_BOOL bPreserveAspectRatio;
   OMX_BOOL bAllowUpscaling;
} __ilpacked__ OMX_PARAM_RESIZETYPE;
/*
The mode determines the kind of resize. \code{OMX_RESIZE_BOX} allow
the \code{nMaxWidth} and \code{nMaxHeight} to set a bounding box into
which the output must fit. \code{OMX_RESIZE_BYTES} allows
\code{nMaxBytes} to set the maximum number of bytes into which the
full output frame must fit.  Two flags aid the setting of the output
size. \code{bPreseveAspectRatio} sets whether the resize should
preserve the aspect ratio of the incoming
image. \code{bAllowUpscaling} sets whether the resize is allowed to
increase the size of the output image compared to the size of the
input image.
*/

typedef struct OMX_PARAM_TESTINTERFACETYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_BOOL bTest;
   OMX_BOOL bSetExtra;
   OMX_U32 nExtra;
} __ilpacked__ OMX_PARAM_TESTINTERFACETYPE;

/* OMX_IndexConfigVisualisation: Visualisation Mode */
typedef struct OMX_CONFIG_VISUALISATIONTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_U8 name[16];
   OMX_U8 property[64];
} __ilpacked__ OMX_CONFIG_VISUALISATIONTYPE;

/*
\code{name} is a string of characters specifying the type of
visualization. The component appends \code{"_vis.vll"} to the name
provided, and attempts to load a visualisation library contained in
this VLL.  \code{property} contains configuration parameters and
values, which is interpreted by the visualisation library. Typically
all visualisations will accept a property string containing
\code{'mode=<number>'}, where \code{<number>} may be a random 32 bit
integer in decimal format. If provided, this may select a random mode
from that visualisation library.
*/

/* OMX_IndexParamImagePoolDisplayFunction: Proprietary Image Passing */
typedef struct OMX_PARAM_IMAGEDISPLAYFUNCTIONTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_PTR pPrivate;
   OMX_PTR (*pDisplayFunc)(void);
   OMX_PTR (*pFlushFunc)(void);
} __ilpacked__ OMX_PARAM_IMAGEDISPLAYFUNCTIONTYPE;
/*
This parameter is used when creating proprietary communication with
the display component, and provides the display function for passing
images to be displayed, together with a function used to flush all
pending image updates and release all images.
*/

/* OMX_IndexParamAudioHandle: Proprietary Audio Handle */
typedef struct OMX_CONFIG_AUDIOHANDLETYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_PTR pHandle;
} __ilpacked__ OMX_CONFIG_AUDIOHANDLETYPE;
/*
This config is used when creating proprietary communication with the
audio render component. It provides the audio handle for getting the
spectrum of a number of samples, used by some visualisations.
*/

/* OMX_IndexParamAudioChannel: Audio Channel Label */
typedef struct OMX_PARAM_AUDIOCHANNELTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nChannel;
   OMX_U32 nFollow;
} __ilpacked__ OMX_PARAM_AUDIOCHANNELTYPE;
/*
The platform-specific audio output device, audioplus, uses a channel
identifier to control audio streams. When sending audio, the
audio_render component will use its own channel identifier. The
identifier is a 4CC, and defaults to \code{'ilar'}. To change this
label, fill the \code{nChannel} field.  The \code{nFollow} field is a
placeholder for later development.
*/

/* OMX_IndexConfigPlayMode: Play Mode */
typedef enum OMX_PLAYMODETYPE {
   OMX_PLAYMODE_NORMAL,
   OMX_PLAYMODE_FF,
   OMX_PLAYMODE_REW,
   OMX_PLAYMODE_DUMMY = 0x7FFFFFFF,
} OMX_PLAYMODETYPE;

typedef struct OMX_CONFIG_PLAYMODETYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_PLAYMODETYPE eMode;
} __ilpacked__ OMX_CONFIG_PLAYMODETYPE;
/*
The playmode affects which frames are extracted from the media file
and passed on the output ports. \code{OMX_PLAYMODE_NORMAL} will
extract all frames, \code{OMX_PLAYMODE_FF} extracts only IDR frames
when video is present, or only occasional packets of audio if no video
is present. \code{OMX_PLAYMODE_REW} is similar to
\code{OMX_PLAYMODE_FF} but extracts packets in reverse time
order.
*/

typedef enum OMX_DELIVERYFORMATTYPE {
   OMX_DELIVERYFORMAT_STREAM,         // no framing information is known
   OMX_DELIVERYFORMAT_SINGLE_PACKET,  // packetised, at most one frame per buffer
   OMX_DELIVERYFORMAT_DUMMY = 0x7FFFFFFF,
} OMX_DELIVERYFORMATTYPE;

typedef struct OMX_PARAM_DELIVERYFORMATTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_DELIVERYFORMATTYPE eFormat;
} __ilpacked__ OMX_PARAM_DELIVERYFORMATTYPE;

/* OMX_IndexParamCodecConfig: Codec Configuration */

typedef struct OMX_PARAM_CODECCONFIGTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_U8 nData[1];
} __ilpacked__ OMX_PARAM_CODECCONFIGTYPE;

/*
This parameter contains opaque data in a format specified by Broadcom
and allows out-of-band information such as cropping rectangles, aspect
ratio information, codec-specific header bytes, and other essential
information to be passed between connected components.
*/

/* OMX_IndexParamCameraPoolToEncoderFunction: Camera and Video Encoder communication */
typedef struct OMX_PARAM_CAMERAPOOLTOENCODERFUNCTIONTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_PTR pPrivate;
   OMX_PTR (*pEncoderFunc)(void);
   OMX_PTR (*pFlushFunc)(void);
   OMX_BOOL bUseYUVUV;
} __ilpacked__ OMX_PARAM_CAMERAPOOLTOENCODERFUNCTIONTYPE;
/*
This parameter is used when creating proprietary communication with
the video_encode component from the capture port. It provides the
encoder function for passing the main and subsampled images to be
encoded, together with a function used to release all images. The
image format in this mode can be set to the Broadcom-specific format
required for the video encode functions, and does not comply with any
of the OMX standard formats.
*/

typedef struct OMX_PARAM_STILLSFUNCTIONTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_BOOL bBuffer;
   OMX_PTR (*pOpenFunc)(void);
   OMX_PTR (*pCloseFunc)(void);
   OMX_PTR (*pReadFunc)(void);
   OMX_PTR (*pSeekFunc)(void);
   OMX_PTR (*pWriteFunc)(void);
} __ilpacked__ OMX_PARAM_STILLSFUNCTIONTYPE;

/* OMX_IndexParamCameraCdiId: Camera Device Specification */
typedef struct OMX_PARAM_CAMERACDIIDTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U8 id[128];
} __ilpacked__ OMX_PARAM_CAMERACDIIDTYPE;
/*
This parameter is used to select which Camera Device Interface (CDI)
is used by Camplus. This specifies the specific camera interface that
is to be used by the camera component, and should take a value as
defined by Broadcom. It currently defaults to the correct value for
the camera mounted on the 2727DK board.
*/
typedef struct OMX_PARAM_BUFFERADDRESSTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_U32 nAllocLen;
   OMX_PTR pBuffer;
} __ilpacked__ OMX_PARAM_BUFFERADDRESSTYPE;

typedef struct OMX_PARAM_TUNNELSETUPTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_TUNNELSETUPTYPE sSetup;
} __ilpacked__  OMX_PARAM_TUNNELSETUPTYPE;

/* OMX_IndexConfigCommonImageFilterParameters: Parameterized Image Filter */
typedef struct OMX_CONFIG_IMAGEFILTERPARAMSTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_IMAGEFILTERTYPE eImageFilter;
   OMX_U32 nNumParams;
   OMX_U32 nParams[4];
} __ilpacked__ OMX_CONFIG_IMAGEFILTERPARAMSTYPE;
/*
This structure contains optional parameters for some image
filters. The following table lists all image filters that support
parameters.

\begin{longtable}{p{6cm}p{3cm}p{7cm}}
\toprule
Filter & Parameters & Notes \\ \midrule

\code{OMX_ImageFilterSolarize} & \code{[<x0> <y0> <y1> <y2>]} &
Linear mapping of \code{[0,<x0>]} to \code{[0,<y0>]}
and \code{[<x0>,255]} to \code{[<y1>,<y2>]}.
Default is \code{"128 128 128 0"}. \\ \midrule

\code{OMX_ImageFilterSharpen} & \code{[<sz> [<str> [<th>]]} &
\code{<sz>} size of filter, either 1 or 2.
\code{<str>} strength of filter.
\code{<th>} threshold of filter.
Default is \code{"1 40 20"}. \\ \midrule

\code{OMX_ImageFilterFilm} & \code{[[<str>] [<u> <v>]]} &
\code{<str>} strength of effect.
\code{<u>} sets u to constant value.
\code{<v>} sets v to constant value.
Default is \code{"24"}. \\ \midrule

\code{OMX_ImageFilterBlur} & \code{[<sz>]} &
\code{<sz>} size of filter, either 1 or 2.
Default is \code{"2"}. \\ \midrule

\code{OMX_ImageFilterSaturation} & \code{[<str>]} &
\code{<str>} strength of effect, in 8.8 fixed point format. u/v value
differences from 128 are multiplied by \code{<str>}.
Default is \code{"272"}. \\ \bottomrule

\end{longtable}
*/


/* OMX_IndexConfigTransitionControl: Transition Control */
typedef struct OMX_CONFIG_TRANSITIONCONTROLTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_U32 nPosStart;
   OMX_U32 nPosEnd;
   OMX_S32 nPosIncrement;
   OMX_TICKS nFrameIncrement;
   OMX_BOOL bSwapInputs;
   OMX_U8 name[16];
   OMX_U8 property[64];
} __ilpacked__ OMX_CONFIG_TRANSITIONCONTROLTYPE;
/*
This structure represents the internal configuration of the
transition. Transitions are generated by a loadable plug-in described
by the \code{name} field. The exact type of transition is configured
in a plug-in-dependent manner with the \code{property} field. All
plug-ins should accept a \code{property} field equal to
\code{"flags=<number>"}.  If \code{bSwapInputs} is false, then the
start image is on port 210, the stop image on port 211. These are
reversed if \code{bSwapInputs} is true.

Transition frames are generated from the plug-in by referencing a
frame position in [0,65536], where position 0 is the start image,
position 65536 is the stop image. The first frame position generated
is \code{nPosStart}. The last frame position generated is
\code{nPosEnd}. Each frame will increment the position by
\code{nPosIncrement}. The timestamp attached to each frame will
increment by \code{nFrameIncrement}.
*/


/* OMX_IndexConfigVEReleaseCallback: Video Encoder Release Events */
typedef struct OMX_PARAM_VIDEOENCODERELEASEFUNCTIONTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_PTR (*pReleaseFunc)(void);
} __ilpacked__ OMX_PARAM_VIDEOENCODERELEASEFUNCTIONTYPE;
/*
This parameter is used to provide a callback function pointer for
release events. It is used for internal clients on VideoCore.
*/


/* OMX_IndexConfigAudioMonoTrackControl: Dual Mono Control */
typedef enum OMX_AUDIOMONOTRACKOPERATIONSTYPE {
   OMX_AUDIOMONOTRACKOPERATIONS_NOP,
   OMX_AUDIOMONOTRACKOPERATIONS_L_TO_R,
   OMX_AUDIOMONOTRACKOPERATIONS_R_TO_L,
   OMX_AUDIOMONOTRACKOPERATIONS_DUMMY = 0x7FFFFFFF,
} OMX_AUDIOMONOTRACKOPERATIONSTYPE ;

typedef struct OMX_CONFIG_AUDIOMONOTRACKCONTROLTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_AUDIOMONOTRACKOPERATIONSTYPE eMode;
} OMX_CONFIG_AUDIOMONOTRACKCONTROLTYPE;
/*
This config controls the options to support dual mono audio
streams. The output can be unchanged, or the left channel copied over
the right channel, or the right channel copied over the left
channel. This config can be applied at any time with stereo
16-bit-per-sample data. Since audio output is typically buffered, any
change will not be audible until that buffering has been played out.
*/


/* OMX_IndexParamCameraImagePool: Camera Image Pools */
typedef struct OMX_PARAM_CAMERAIMAGEPOOLTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 num_hi_res_frames;
   OMX_U32 hi_res_width;
   OMX_U32 hi_res_height;
   OMX_COLOR_FORMATTYPE hi_res_type;
   OMX_U32 num_low_res_frames;
   OMX_U32 low_res_width;
   OMX_U32 low_res_height;
   OMX_COLOR_FORMATTYPE low_res_type;
   OMX_U32 num_input_frames;
   OMX_U32 input_width;
   OMX_U32 input_height;
   OMX_COLOR_FORMATTYPE input_type;
} __ilpacked__ OMX_PARAM_CAMERAIMAGEPOOLTYPE;
/*
\sloppy This parameter specifies the size, type, and number, of images to
allow in the images pools required by Camplus. Supported types are
\code{OMX_COLOR_FormatYUV420PackedPlanar}, 
\code{OMX_COLOR_FormatYUV422PackedPlanar}, 
\code{OMX_COLOR_FormatRawBayer8bit}, 
\code{OMX_COLOR_FormatRawBayer10bit}, 
\code{OMX_COLOR_FormatRawBayer8bitcompressed}, and 0 (reserved for the
Broadcom-specific format required by the video encoder). The input
pool width, height, and type can be set as 0 to make the component
query Camplus for the sensor mode that would correspond to the largest
of the viewfinder port definition, the capture port definition, or the
high resolution image pool.
*/

/* OMX_IndexParamImagePoolSize: Specifying Image Pool Properties */
typedef struct OMX_PARAM_IMAGEPOOLSIZETYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 width;
   OMX_U32 height;
   OMX_U32 num_pages;
} __ilpacked__ OMX_PARAM_IMAGEPOOLSIZETYPE;
/*
This parameter is used to control the size of pool that the component
will allocate in the absence of setting an external pool.  The default
can be reset by setting this parameter with all three fields set to
zero.
*/


/* OMX_IndexParamImagePoolExternal: Providing Allocated Image Pools */
typedef struct OMX_PARAM_IMAGEPOOLEXTERNALTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_PTR image_pool;
   OMX_PTR image_pool2;
   OMX_PTR image_pool3;
} __ilpacked__ OMX_PARAM_IMAGEPOOLEXTERNALTYPE;
/*
This parameter is used by clients on VideoCore that want to control
memory usage by providing an allocated image pool for the component to
use.  If all image_pool pointers are NULL, the default value, then the
component allocates its own pool.
*/

typedef struct OMX_PARAM_CAMERASTRIPEFUNCTIONTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_PTR pPrivate;
   OMX_PTR (*pStripeFunc)(void);
   OMX_PTR (*pFlushFunc)(void);
   OMX_PTR (*pReleaseFunc)(void);
   OMX_PTR pReleasePrivate;
   OMX_BOOL bUseYUVUV;
} __ilpacked__ OMX_PARAM_CAMERASTRIPEFUNCTIONTYPE;

typedef struct OMX_PARAM_RUTILFIFOINFOTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_PTR pILFifo;
} __ilpacked__ OMX_PARAM_RUTILFIFOINFOTYPE;

typedef struct OMX_PARAM_ILFIFOCONFIG {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_U32 nDataSize;
   OMX_U32 nHeaderCount;
} __ilpacked__ OMX_PARAM_ILFIFOCONFIG;

/* OMX_IndexConfigCameraSensorModes: Camera Sensor Mode */
typedef struct OMX_CONFIG_CAMERASENSORMODETYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_U32 nModeIndex;
   OMX_U32 nNumModes;
   OMX_U32 nWidth;
   OMX_U32 nHeight;
   OMX_U32 nPaddingRight;
   OMX_U32 nPaddingDown;
   OMX_COLOR_FORMATTYPE eColorFormat;
   OMX_U32 nFrameRateMax;
   OMX_U32 nFrameRateMin;
} __ilpacked__ OMX_CONFIG_CAMERASENSORMODETYPE;
/*
This parameter is used by clients to determine the sensor mode, and
hence the memory usage, of the camera module. This is primarily used
for determining the size of the input image pool.

It can be used in two ways dependent on \code{nPortIndex}. If
\code{nPortIndex} is \code{OMX_ALL}, it returns the sensor mode
corresponding to \code{nModeIndex}, and the number of modes in
\code{nNumModes}. If \code{nModeIndex} is greater than or equal to
\code{nNumModes} only \code{nNumModes} is returned. If
\code{nPortIndex} is equal to a camera video output port index, it
returns the sensor mode that would be selected for the values
currently in \code{OMX_IndexParamPortDefinition} for that port.

The \code{nPaddingRight} and \code{nPaddingDown} values determine the
extra padding the sensor adds to the image. These values must be added
to \code{nWidth} and \code{nHeight} respectively if the client is
specifying the input image pool size.
*/

typedef struct OMX_CONFIG_BRCMPORTSTATS {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_U32 nImageCount;
   OMX_U32 nBufferCount;
   OMX_U32 nFrameCount;
} __ilpacked__ OMX_CONFIG_BRCMPORTSTATS;

typedef struct OMX_CONFIG_BRCMCAMERASTATS {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nOutFrameCount;
   OMX_U32 nDroppedFrameCount;
} OMX_CONFIG_BRCMCAMERASTATS;

typedef struct OMX_CONFIG_SHARPNESSTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_S32 nSharpness;
} __ilpacked__ MX_CONFIG_SHARPNESSTYPE;

typedef enum OMX_COMMONFLICKERCANCELTYPE {
   OMX_COMMONFLICKERCANCEL_OFF,
   OMX_COMMONFLICKERCANCEL_AUTO,
   OMX_COMMONFLICKERCANCEL_50,
   OMX_COMMONFLICKERCANCEL_60,
   OMX_COMMONFLICKERCANCEL_DUMMY = 0x7FFFFFFF,
} OMX_COMMONFLICKERCANCELTYPE ;

typedef struct OMX_CONFIG_FLICKERCANCELTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_COMMONFLICKERCANCELTYPE eFlickerCancel;
} __ilpacked__ OMX_CONFIG_FLICKERCANCELTYPE;

typedef enum OMX_REDEYEREMOVALTYPE {
   OMX_RedEyeRemovalNone,                           /**< No red eye removal */
   OMX_RedEyeRemovalOn,                             /**< Red eye removal on */
   OMX_RedEyeRemovalAuto,                           /**< Red eye removal will be done automatically when detected */
   OMX_RedEyeRemovalKhronosExtensions = 0x6F000000, /**< Reserved region for introducing Khronos Standard Extensions */ 
   OMX_RedEyeRemovalVendorStartUnused = 0x7F000000, /**< Reserved region for introducing Vendor Extensions */
   OMX_RedEyeRemovalMax = 0x7FFFFFFF
} OMX_REDEYEREMOVALTYPE;

typedef struct OMX_CONFIG_REDEYEREMOVALTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_REDEYEREMOVALTYPE eMode;
} __ilpacked__ OMX_CONFIG_REDEYEREMOVALTYPE;

typedef enum OMX_FACEDETECTIONCONTROLTYPE {
   OMX_FaceDetectionControlNone,                           /**< Disables face detection */
   OMX_FaceDetectionControlOn,                             /**< Enables face detection */
   OMX_FaceDetectionControlKhronosExtensions = 0x6F000000, /**< Reserved region for introducing Khronos Standard Extensions */ 
   OMX_FaceDetectionControlVendorStartUnused = 0x7F000000, /**< Reserved region for introducing Vendor Extensions */
   OMX_FaceDetectionControlMax = 0x7FFFFFFF
} OMX_FACEDETECTIONCONTROLTYPE;

typedef struct OMX_CONFIG_FACEDETECTIONCONTROLTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;
   OMX_FACEDETECTIONCONTROLTYPE eMode;
} __ilpacked__ OMX_CONFIG_FACEDETECTIONCONTROLTYPE;

typedef struct OMX_FACEREGIONTYPE {
   OMX_S16 nLeft;              /**< X Coordinate of the top left corner of the rectangle */
   OMX_S16 nTop;               /**< Y Coordinate of the top left corner of the rectangle */
   OMX_U16 nWidth;             /**< Width of the rectangle */
   OMX_U16 nHeight;            /**< Height of the rectangle */
} __ilpacked__ OMX_FACEREGIONTYPE;

typedef struct OMX_CONFIG_FACEDETECTIONREGIONTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;            /**< index of port with face detection enabled */
   OMX_U32 nIndex;                /**< first requested region number, allowing retrieval of many regions
                                       over several requests */
   OMX_U32 nDectectedRegions;     /**< total number of detected regions */
   OMX_S32 nValidRegions;         /**< number of valid regions in sRegion array
                                       When getting, the client sets this to the number of regions available.
                                       The component writes region data and updates this field with how many
                                       regions have been written to. */
   OMX_FACEREGIONTYPE sRegion[1];  /**< variable length array of face regions */
} __ilpacked__ OMX_CONFIG_FACEDETECTIONREGIONTYPE;

typedef enum OMX_INTERLACETYPE {
   OMX_InterlaceProgressive,                    /**< The data is not interlaced, it is progressive scan */
   OMX_InterlaceFieldSingleUpperFirst,          /**< The data is interlaced, fields sent
                                                     separately in temporal order, with upper field first */
   OMX_InterlaceFieldSingleLowerFirst,          /**< The data is interlaced, fields sent
                                                     separately in temporal order, with lower field first */
   OMX_InterlaceFieldsInterleavedUpperFirst,    /**< The data is interlaced, two fields sent together line
                                                     interleaved, with the upper field temporally earlier */
   OMX_InterlaceFieldsInterleavedLowerFirst,    /**< The data is interlaced, two fields sent together line
                                                     interleaved, with the lower field temporally earlier */
   OMX_InterlaceMixed,                          /**< The stream may contain a mixture of progressive 
                                                     and interlaced frames */
   OMX_InterlaceKhronosExtensions = 0x6F000000, /**< Reserved region for introducing Khronos Standard Extensions */ 
   OMX_InterlaceVendorStartUnused = 0x7F000000, /**< Reserved region for introducing Vendor Extensions */
   OMX_InterlaceMax = 0x7FFFFFFF
} OMX_INTERLACETYPE;

typedef struct OMX_CONFIG_INTERLACETYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U32 nPortIndex;            /**< index of port emitting or accepting the content */
   OMX_INTERLACETYPE eMode;       /**< The interlace type of the content */
   OMX_BOOL bRepeatFirstField;    /**< Whether to repeat the first field */
} __ilpacked__ OMX_CONFIG_INTERLACETYPE;

/* OMX_IndexParamIspTuner: Custom ISP tuner */
typedef struct OMX_PARAM_CAMERAISPTUNERTYPE {
   OMX_U32 nSize;
   OMX_VERSIONTYPE nVersion;
   OMX_U8 tuner_name[64];
} __ilpacked__ OMX_PARAM_CAMERAISPTUNERTYPE;
/*
This parameter allows a custom ISP tuner to be loaded instead of
the default one specified for the camera module. Setting an empty 
string uses the default value.
*/


#endif
/* File EOF */
