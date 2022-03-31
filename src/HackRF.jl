module HackRF

using Printf
using Libdl
# using Hackrf_jll # TODO

LIBHACKRF = nothing
LIBUSB = nothing

DEVICE_LIST = nothing

function __init__()
    global LIBHACKRF
    global LIBUSB
    global DEVICE_LIST
    function openlib(name)
        libname = Libdl.find_library(name)
        if length(libname) == 0
            error("Could not find $name on this system")
        end
        return Libdl.dlopen(libname)
    end
    LIBHACKRF = openlib("libhackrf")
    LIBUSB = openlib("libusb-1.0")
    @info "libhackrf version $(hackrf_library_version()) ($(hackrf_library_release()))"
    @info "Initializing HackRF Device. If it is not connected, this function will fail."
    try
        init()
        DEVICE_LIST = DeviceList()
    catch e
        @warn "Failed to initialize HackRF and build device list: $e"
    end
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

const DevicePtr = Ptr{Cvoid} # typedef struct hackrf_device hackrf_device

struct DeviceX
    serial::String
end

function Base.getindex(d::DeviceX, field::Symbol)
    if field === :serial 
        return d.serial
    else
        for i in 1:DEVICE_LIST.devicecount
            if DEVICE_LIST.serial_numbers[i] == serial
                return getfield(DEVICE_LIST, field)[i]
            end
        end
        error("Device with serial number $(d.serial) not found in DEVICE_LIST")
    end
end



mutable struct Device
    pointer::DevicePtr
    usb_device::DevicePtr
    usb_board_id::UsbBoardId.t
    serial_number::String
    status::Bool

    function Device(;pointer=DevicePtr(), usb_device, usb_board_id, serial_number, status=false)
        ## Now, we want to make sure that the C data is properly freed when this
        ## struct is garbage-collected, so let's define a finalizer
        #function device_finalizer(dev::Device)
        #    if dev.status
        #        @async @info "Closing $(dev.usb_board_id) device $(dev.serial_number)"
        #        try 
        #            close(dev)
        #        catch e
        #            true
        #        end
        #    end
        #end

        dev = new(pointer, usb_device, usb_board_id, serial_number, status)
        #finalizer(device_finalizer, dev)
        dev
    end
end


function Base.:(==)(a::Device, b::Device)
    (a.pointer == b.pointer &&
     a.usb_device == b.usb_device &&
     a.serial_number == b.serial_number &&
     a.usb_board_id == b.usb_board_id)
end



#
# USB transfer information passed to RX or TX callback.
# A callback should treat all these fields as read-only except that a TX callback should write to the data buffer.
#
struct Transfer
    device::DevicePtr          # HackRF USB device for this transfer
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
    serial_number::Vector{String}
    usb_board_id::Vector{UsbBoardId.t}
    usb_device_index::Vector{Int}
    devicecount::Int
    usb_device::Vector{Ptr{Cvoid}}
    status::Vector{Bool}
    pointer::Vector{DevicePtr}
    device_list_pointer::Ptr{Cvoid}
    
    function DeviceList()
        global DEVICE_LIST # really should be a singleton
        ## Call the hackrf_device_list() function from the C library
        h_hackrf_device_list = dlsym(LIBHACKRF, :hackrf_device_list)
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
        usb_device = unsafe_wrap(Vector{Ptr{Cvoid}}, unsafe_load(Ptr{Ptr{Ptr{Cvoid}}}(ptr),5), devicecount)
        devs = new(
            serial_numbers,
            usb_board_ids,
            usb_device_index,
            devicecount,
            usb_device,
            zeros(Bool, devicecount),
            [DevicePtr() for _ in 1:devicecount],
            Ptr{Cvoid}(ptr),
        )

        ## Now, we want to make sure that the C data is properly freed when this
        ## struct is garbage-collected, so let's define a finalizer
        #function device_list_finalizer(devs::DeviceList)
        #    @async @info "Freeing Device List"
        #    h_hackrf_device_list_free = dlsym(LIBHACKRF, :hackrf_device_list_free)
        #    ccall(h_hackrf_device_list_free, Cvoid, (Ptr{Cvoid},), devs.opaque_ptr)
        #    for i in 1:devs.devicecount
        #        if devs.status[i]
        #            @async @info "Closing device $i"
        #            device_list_close(devs, i)
        #        end
        #    end
        #end
        #finalizer(device_list_finalizer, devs)

        ## And we're done. Return the struct.
        if DEVICE_LIST === nothing || devs.devicecount != DEVICE_LIST.devicecount
            DEVICE_LIST = devs
        end
        return devs
    end
end

export devices
devices() = device_list_to_list_of_devices(DeviceList())

function Base.show(io::IO, devs::DeviceList)
    @printf(io, "Index\t%-24s\t%-32s\t%s\n", "Board ID", "Serial Number", "USB")
    for i in 1:devs.devicecount
        d = devs[i]
        @printf(io, "%.2d\t%s", i, d) 
    end
end

function Base.show(io::IO, dev::Device)
    status(x::Bool) = x ? "OPEN (running firmware $(firmware_version(dev)))" : "CLOSED"
    @printf(io, "%-24s\t%-32s\tPort %.3o\t%s\n",
        dev.usb_board_id,
        dev.serial_number,
        usb_port_number(dev.usb_device),
        status(dev.status),
    )
end

function device_list_to_list_of_devices(d::DeviceList)::Vector{Device}
    [Device(;pointer=d.pointer[i], 
            usb_device=d.usb_device[i], 
            serial_number=d.serial_numbers[i], 
            usb_board_id=d.usb_board_ids[i],
            status=d.status[i])
    for i in 1:d.devicecount] 
end

function device_list_open(devs::DeviceList, index::Integer)
    if devs.status[index]
        @info "Device $index is already open"
        return devs[index]
    end
    device_ptr = Ref{DevicePtr}()
    h_hackrf_device_list_open = dlsym(LIBHACKRF, :hackrf_device_list_open)
    result = ccall(h_hackrf_device_list_open, 
                Cint, 
                (Ptr{Cvoid}, Cint, DevicePtr),
                devs.opaque_ptr,
                Cint(index-1),
                device_ptr) |> Error.t
    if result !== Error.HACKRF_SUCCESS
        error(result)
    end
    devs.status[index] = true
    devs.pointer[index] = device_ptr[]
    return devs[index]
end

function device_list_open(index::Integer)
    if DEVICE_LIST === nothing
        DeviceList() |> print
    end
    device_list_open(DEVICE_LIST, index)
end

function device_list_close(devs::DeviceList, index::Integer)
    if !devs.status[index]
        @info "Device $index is already closed"
        return devs.pointer[index]
    end

    h_hackrf_close = dlsym(LIBHACKRF, :hackrf_close)
    device = devs.pointer[index]
    result = ccall(h_hackrf_close,
        Cint,
        (Ptr{DevicePtr},),
        device) |> Error.t
    if result !== Error.HACKRF_SUCCESS
        error(result)
    end
    devs.status[index] = false
    devs.pointer[index] = DevicePtr()
    return devs.pointer[index]
end    


function usb_port_number(usb_dev::Ptr{Cvoid})
    libusb_get_port_number = dlsym(LIBUSB, :libusb_get_port_number)
    ccall(libusb_get_port_number, UInt8, (Ptr{Cvoid},), usb_dev)
end

function Base.isempty(devs::DeviceList)
    devs.devicecount == 0
end

## A few convenience functions to make the DeviceList behave more like a "list"

Base.length(devs::DeviceList) = devs.devicecount

Base.in(serial_number::String, devs::DeviceList) = serial_number ∈ devs.serial_numbers

Base.iterate(devs::DeviceList) = Base.iterate(device_list_to_list_of_devices(devs))

Base.iterate(devs::DeviceList, state) = Base.iterate(device_list_to_list_of_devices(devs), state)
    
Base.getindex(devs::DeviceList, key...) = Base.getindex(device_list_to_list_of_devices(devs), key...)

function Base.setindex!(devs::DeviceList, d::Device, i)
    for i in 1:devs.devicecount
        if devs.serial_numbers[i] == d.serial_number
            devs.status[i] = d.status
            devs.pointer[i] = d.pointer
            devs.usb_device[i] = d.usb_device
            devs.usb_board_ids[i] = d.usb_board_id
        end
    end
end



#########
## API ##
#########

function check(result::Error.t)
    if result !== Error.HACKRF_SUCCESS
        error(result)
    end
    return result
end

function check(result::Cint)
    check(Error.t(result))
end

function init()
    h_hackrf_init = dlsym(LIBHACKRF, :hackrf_init)
    return ccall(h_hackrf_init, Cint, ()) |> check
end

function exit()
    h_hackrf_exit = dlsym(LIBHACKRF, :hackrf_exit)
    return ccall(h_hackrf_exit, Cint, ()) |> check
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

export open
function open(d::Device; reset_if_open=true)
    open_dev = open(d.serial_number; reset_if_open)
    d.pointer = open_dev.pointer
    d.usb_device = open_dev.usb_device
    d.status = open_dev.status
    return d |> update_device_list
end

function open(serial::String; reset_if_open=true)
    devs = DeviceList()
    for i in 1:devs.devicecount
        if devs[i].serial_number == serial
            return device_list_open(devs, i) |> update_device_list
        end
    end
    error("Device with serial number $serial not found")
end


export close
function close(d::Device)
    h_hackrf_close = dlsym(LIBHACKRF, :hackrf_close)
    devptr = d.pointer
    ccall(h_hackrf_close, Cint, (DevicePtr,), devptr) |> check
    #d.pointer = devptr[]
    d.status = false
    return d |> update_device_list
end

export reset
function reset(d::Device)
    if d.pointer == DevicePtr() # NULL pointer check
        error("Device pointer is NULL!")
    end
    h_hackrf_reset = dlsym(LIBHACKRF, :hackrf_reset)
    ccall(h_hackrf_reset, Cint, (DevicePtr,), d.pointer) |> check
    d.status = false
    return d |> update_device_list
end

function update_device_list(d::Device)
    global DEVICE_LIST
    for i in 1:DEVICE_LIST.devicecount
        if DEVICE_LIST.serial_numbers[i] == d.serial_number
            @info "Updating device $(d.serial_number) in DEVICE_LIST"
            DEVICE_LIST[i] = d
        end
    end
    return d
end

function firmware_version(d::Device)
    buf = zeros(UInt8, 255)
    bufptr = Base.unsafe_convert(Ptr{Cchar}, buf)
    ccall(dlsym(LIBHACKRF, :hackrf_version_string_read), Cint, (DevicePtr, Ptr{Cchar}, Cint),
        d.pointer, bufptr, 255) |> check
    firmware = unsafe_string(bufptr)
    usb_ver = Ref(0x0000)
    ccall(dlsym(LIBHACKRF, :hackrf_usb_api_version_read), Cint, (DevicePtr, Ptr{Cushort}),
        d.pointer, usb_ver) |> check
    usb_ver_str = @sprintf("%x.%02x", (usb_ver[] >> 8) & 0xFF, usb_ver[] & 0xFF)
    return "$(firmware) (API:$usb_ver_str)"
end

export set_sample_rate
function set_sample_rate(d::Device, hz::Number)
    ccall(dlsym(LIBHACKRF, :hackrf_set_sample_rate),
        Cint,
        (DevicePtr, Cdouble),
        d.pointer,
        Cdouble(hz)) |> check
end

end # module
