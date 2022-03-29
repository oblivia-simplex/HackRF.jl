module HackRF

using Printf
using Libdl
# using Hackrf_jll # TODO

LIBHACKRF = nothing
LIBUSB = nothing

function __init__()
    global LIBHACKRF
    global LIBUSB
    function openlib(name)
        libname = Libdl.find_library(name)
        if length(libname) == 0
            error("Could not find $name on this system")
        end
        return Libdl.dlopen(libname)
    end
    LIBHACKRF = openlib("libhackrf")
    LIBUSB = openlib("libusb-1.0")

end 


module Error 
@enum t begin
	HACKRF_SUCCESS = 0
	HACKRF_TRUE = 1
	HACKRF_ERROR_INVALID_PARAM = -2
	HACKRF_ERROR_NOT_FOUND = -5
	HACKRF_ERROR_BUSY = -6
	HACKRF_ERROR_NO_MEM = -11
	HACKRF_ERROR_LIBUSB = -1000
	HACKRF_ERROR_THREAD = -1001
	HACKRF_ERROR_STREAMING_THREAD_ERR = -1002
	HACKRF_ERROR_STREAMING_STOPPED = -1003
	HACKRF_ERROR_STREAMING_EXIT_CALLED = -1004
	HACKRF_ERROR_USB_API_VERSION = -1005
	HACKRF_ERROR_NOT_LAST_DEVICE = -2000
	HACKRF_ERROR_OTHER = -9999
end
end #end HackRFError

module BoardId
@enum t begin
	BOARD_ID_JELLYBEAN  = 0
	BOARD_ID_JAWBREAKER = 1
	BOARD_ID_HACKRF_ONE = 2
	BOARD_ID_RAD1O = 3
	BOARD_ID_INVALID = 0xFF
end
end #end BoardID

module UsbBoardId
@enum t begin
    USB_BOARD_ID_JAWBREAKER = 0x604B
	USB_BOARD_ID_HACKRF_ONE = 0x6089
	USB_BOARD_ID_RAD1O = 0xCC15
	USB_BOARD_ID_INVALID = 0xFFFF
end
end #end UsbBoardId

module OperaCakePorts
@enum t begin
    OPERACAKE_PA1 = 0
	OPERACAKE_PA2 = 1
	OPERACAKE_PA3 = 2
	OPERACAKE_PA4 = 3
	OPERACAKE_PB1 = 4
	OPERACAKE_PB2 = 5
	OPERACAKE_PB3 = 6
	OPERACAKE_PB4 = 7
end
end #end OperaCakePorts

module OperaCakeSwitchingMode
@enum t begin
    OPERACAKE_MODE_MANUAL = 0
    OPERACAKE_MODE_FREQUENCY = 1
    OPERACAKE_MODE_TIME = 2
end
end # end OperaCakeSwitchingMode

module SweepStyle
@enum t begin
    LINEAR = 0
    INTERLEAVED = 1
end
end # end SweepStyle

const Device = Ptr{Cvoid} # typedef struct hackrf_device hackrf_device

#
# USB transfer information passed to RX or TX callback.
# A callback should treat all these fields as read-only except that a TX callback should write to the data buffer.
#
struct Transfer
    device::Device          # HackRF USB device for this transfer
    buffer::Vector{UInt8}   # transfer data buffer
    buffer_len::Int         # length of data buffer in bytes
    valid_len::Int          # number of bytes that were transferred
    rx_ctx::Ptr{Cvoid}      # RX libusb context
    tx_ctx::Ptr{Cvoid}      # TX libusb context
end

struct ReadPartIdSerialNo
    part_id::Tuple{UInt32,UInt32}
    serial_no::Tuple{UInt32,UInt32,UInt32,UInt32}
end

struct OperacakeFreqRange
    freq_min::UInt16
    freq_max::UInt16
    port::UInt8
end

struct M0State
    requested_mode::UInt16
    request_flag::UInt16
    active_mode::UInt32
    m0_count::UInt32
    m4_count::UInt32
    num_shortfalls::UInt32
    longest_shortfall::UInt32
    shortfall_limit::UInt32
    threshold::UInt32
    next_mode::UInt32
    error::UInt32
end


mutable struct DeviceList
    serial_numbers::Vector{String}
    usb_board_ids::Vector{UsbBoardId.t}
    usb_device_index::Vector{Int}
    devicecount::Int
    usb_devices::Vector{Ptr{Cvoid}}
    status::Vector{Bool}
    device::Vector{Device}
    opaque_ptr::Ptr{Cvoid}
    
    function DeviceList()
        ## Call the hackrf_device_list() function from the C library
        h_hackrf_device_list = dlsym(LIBHACKRF, :hackrf_device_list)
        if (err = hackrf_init()) !== Error.HACKRF_SUCCESS
            error(err)
        end
        ptr = ccall(h_hackrf_device_list, Ptr{Ptr{Cvoid}}, ()) 

        ## Now marshall that C data into a nice Julia struct
        devicecount = unsafe_load(Ptr{Int64}(ptr),4) 
        serial_numbers_raw = unsafe_wrap(Vector{Cstring}, 
            Ptr{Cstring}(unsafe_load(Ptr{Ptr{Cstring}}(ptr))),
            devicecount,
            own=false)
        serial_numbers = [unsafe_string(s) for s in serial_numbers_raw]
        usb_board_ids_ints = unsafe_wrap(Vector{UInt64}, unsafe_load(Ptr{Ptr{UInt64}}(ptr),2), devicecount)
        usb_board_ids = UsbBoardId.t.(usb_board_ids_ints)
        usb_device_index = unsafe_wrap(Vector{Int64}, unsafe_load(Ptr{Ptr{Int64}}(ptr),3), devicecount)
        usb_devices = unsafe_wrap(Vector{Ptr{Cvoid}}, unsafe_load(Ptr{Ptr{Ptr{Cvoid}}}(ptr),5), devicecount)
        devs = new(
            serial_numbers,
            usb_board_ids,
            usb_device_index,
            devicecount,
            usb_devices,
            zeros(Bool, devicecount),
            [Device() for _ in 1:devicecount],
            Ptr{Cvoid}(ptr)
        )

        ## Now, we want to make sure that the C data is properly freed when this
        ## struct is garbage-collected, so let's define a finalizer
        function device_list_finalizer(devs::DeviceList)
            @async @info "Freeing Device List"
            h_hackrf_device_list_free = dlsym(LIBHACKRF, :hackrf_device_list_free)
            ccall(h_hackrf_device_list_free, Cvoid, (Ptr{Cvoid},), devs.opaque_ptr)
            for i in 1:devs.devicecount
                if devs.status[i]
                    @async @info "Closing device $i"
                    device_list_close(devs, i)
                end
            end
        end
        finalizer(device_list_finalizer, devs)

        ## And we're done. Return the struct.
        return devs
    end
end

function Base.show(io::IO, devs::DeviceList)
    @printf(io, "Index\t%-24s\t%-32s\t%s\n", "Board ID", "Serial Number", "USB")
    status(x::Bool) = x ? "OPEN" : "CLOSED"
    for i in 1:devs.devicecount
        @printf(io, "%.2d.\t%-24s\t%-32s\tPort %.3o\t%s\n", 
                i, 
                devs.usb_board_ids[i], 
                devs.serial_numbers[i], 
                usb_port_number(devs.usb_devices[i]),
                status(devs.status[i]))
    end
end


function device_list_open(devs::DeviceList, index::Integer)
    if devs.status[index]
        @info "Device $index is already open"
        return devs.device[index]
    end
    device_ptr = Ref{Device}()
    h_hackrf_device_list_open = dlsym(LIBHACKRF, :hackrf_device_list_open)
    result = ccall(h_hackrf_device_list_open, 
                Cint, 
                (Ptr{Cvoid}, Cint, Device),
                devs.opaque_ptr,
                Cint(index-1),
                device_ptr) |> Error.t
    if result !== Error.HACKRF_SUCCESS
        error(result)
    end
    devs.status[index] = true
    devs.device[index] = device_ptr[]
    return device_ptr[]
end

function device_list_close(devs::DeviceList, index::Integer)
    if !devs.status[index]
        @info "Device $index is already closed"
        return devs.device[index]
    end

    h_hackrf_close = dlsym(LIBHACKRF, :hackrf_close)
    device = devs.device[index]
    result = ccall(h_hackrf_close,
        Cint,
        (Ptr{Device},),
        device) |> Error.t
    if result !== Error.HACKRF_SUCCESS
        error(result)
    end
    devs.status[index] = false
    devs.device[index] = Device()
    return devs.device[index]
end    


function usb_port_number(usb_dev::Ptr{Cvoid})
    libusb_get_port_number = dlsym(LIBUSB, :libusb_get_port_number)
    ccall(libusb_get_port_number, UInt8, (Ptr{Cvoid},), usb_dev)
end


#########
## API ##
#########

function hackrf_init()
    h_hackrf_init = dlsym(LIBHACKRF, :hackrf_init)
    return ccall(h_hackrf_init, Cint, ()) |> Error.t
end

function hackrf_exit()
    h_hackrf_exit = dlsym(LIBHACKRF, :hackrf_exit)
    return ccall(h_hackrf_exit, Cint, ()) |> Error.t
end

export hackrf_library_version
function hackrf_library_version()
    h_hackrf_library_version = dlsym(LIBHACKRF, :hackrf_library_version)
    return ccall(h_hackrf_library_version, Cstring, ()) |> unsafe_string
end

export hackrf_library_release
function hackrf_library_release()
    h_hackrf_library_release = dlsym(LIBHACKRF, :hackrf_library_release)
    return ccall(h_hackrf_library_release, Cstring, ()) |> unsafe_string
end



end # module
