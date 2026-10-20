# Loaned messages

Publishers and subscriptions support the RMW loan API when Zenoh shared memory is
configured and the message has a compatible fixed memory layout. Applications can
use `rclcpp::Publisher::borrow_loaned_message()` without application changes.
Subscriptions must enable loans with `ROS_DISABLE_LOANED_MESSAGES=0`. The current
Jazzy rclcpp executor returns subscription loans immediately after the callback;
use a `const Message &` callback and do not retain references beyond the callback.
rclcpp disables middleware loans for publishers using intra-process communication
and uses its existing allocator and intra-process path instead.

## Supported layouts

Loan eligibility requires the generated Fast RTPS `PLAIN_TYPE` classification and
C or C++ introspection metadata. The implementation checks every field offset
against CDR alignment, including nested messages and fixed arrays, and requires
that the CDR body size equal `sizeof(message)`. Strings, sequences (including
bounded sequences), empty messages, wide characters, long doubles, and layouts
with trailing padding use the normal serialization path. Missing introspection
support also disables loans. This is deliberately more conservative than simply
checking that a message has a bounded serialized size.

## Wire format and alignment

The wire payload remains standard CDR: a four-byte encapsulation header followed
by the message body. There is no private attachment flag or raw-struct protocol.
A native-endian body with a verified layout is already a valid CDR representation.
Existing publishers, regular and serialized subscribers, and remote peers can
continue using the standard serialization/deserialization path. A receiver with a
different native byte order deserializes into an owned loan buffer.

The header and body are separate Zenoh byte slices. Placing an eight-byte-aligned
ROS struct immediately after a four-byte header in an aligned allocation would
misalign the struct. The body therefore uses an aligned Zenoh SHM allocation and
the header uses a separate small allocation. The receiving payload retains its
slices and only flattens them when a consumer needs a contiguous serialized buffer.
A loaning subscription takes a direct view of the body when its byte order and
alignment permit; otherwise it deserializes into an aligned owned buffer.

The Zenoh transport provider supports byte-aligned allocations. Each context
lazily creates an additional POSIX Zenoh SHM provider with eight-byte alignment for
ROS loans. Its capacity uses the configured transport SHM pool size
(`ZENOH_SHM_ALLOC_SIZE` or `transport/shared_memory/transport_optimization/pool_size`).
This can reserve one additional pool per context; the transport pool is unchanged.
The transport message-size threshold does not restrict explicit message loans.

## Ownership and allocation limits

Borrowed publisher messages are initialized with their generated defaults, with
padding zeroed before initialization. The publisher owns outstanding loans until
return or successful publication. A failed publication leaves a returnable loan.
Publishing transfers a reference to Zenoh; transport queues and transient-local
history retain that reference independently of the original publisher loan.

Taken subscription loans retain the received sample, including its Zenoh byte
references, until `rmw_return_loaned_message_from_subscription()`. Returning one
loan does not invalidate other subscribers or cached samples. Unknown pointers,
wrong-owner returns, and duplicate returns are rejected. Destroying an entity
reclaims outstanding loans; applications must return their loans before destroying
the corresponding entity.

With SHM disabled, initialization failure, or a message too large for the pool,
`can_loan_messages` is false and direct RMW loan operations return unsupported.
rclcpp then uses its existing allocator fallback. If a capable publisher's pool
is temporarily exhausted by outstanding loans, borrowing falls back to an owned
heap buffer. This preserves the loan API and wire format but does not promise
zero copies through a transport in that case. Allocation failure of both SHM and
heap returns `RMW_RET_BAD_ALLOC`.

## Tests and benchmark

`test_rmw_zenoh_cpp` generates a plain message containing a timestamp and a fixed
4 MiB byte array. Its loan tests cover pointer identity, defaults, outstanding
loans, return errors, mixed takes, C type support, nested layouts, padding rejection,
opposite-endian CDR, serialized takes, transient-local history, pool exhaustion,
and SHM disabled or undersized.

The build also provides `loan_benchmark`. Use separate publisher and subscriber
processes for transport measurements:

```sh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=171
export ROS_DISABLE_LOANED_MESSAGES=0
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/localhost:7480"];transport/shared_memory/enabled=true'
# Start rmw_zenohd listening on tcp/localhost:7480 separately.
./build/test_rmw_zenoh_cpp/loan_benchmark plain sub
./build/test_rmw_zenoh_cpp/loan_benchmark plain pub
```

Use `image` in place of `plain` for the non-plain `sensor_msgs/Image` control.
Use `both` in place of `pub`/`sub` for a session-local functional measurement.
Parameters `count` (default 120), `warmup` (10), and `hz` (30) are ROS parameters.
The benchmark uses const-reference callbacks and reports callback-entry latency
p50/p99 and a payload checksum. Preloading `libmemcount.so` measures large
`memcpy`/`memmove` calls; both sides reset its counters after warmup. A zero result
in `both` mode establishes session-local payload behavior, not transport behavior.
