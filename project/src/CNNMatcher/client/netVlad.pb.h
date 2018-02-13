// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: netVlad.proto

#ifndef PROTOBUF_netVlad_2eproto__INCLUDED
#define PROTOBUF_netVlad_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3003000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3003002 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
namespace netVlad {
class NetRequest;
class NetRequestDefaultTypeInternal;
extern NetRequestDefaultTypeInternal _NetRequest_default_instance_;
class NetResponse;
class NetResponseDefaultTypeInternal;
extern NetResponseDefaultTypeInternal _NetResponse_default_instance_;
}  // namespace netVlad

namespace netVlad {

namespace protobuf_netVlad_2eproto {
// Internal implementation detail -- do not call these.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[];
  static const ::google::protobuf::uint32 offsets[];
  static void InitDefaultsImpl();
  static void Shutdown();
};
void AddDescriptors();
void InitDefaults();
}  // namespace protobuf_netVlad_2eproto

// ===================================================================

class NetRequest : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:netVlad.NetRequest) */ {
 public:
  NetRequest();
  virtual ~NetRequest();

  NetRequest(const NetRequest& from);

  inline NetRequest& operator=(const NetRequest& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const NetRequest& default_instance();

  static inline const NetRequest* internal_default_instance() {
    return reinterpret_cast<const NetRequest*>(
               &_NetRequest_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(NetRequest* other);

  // implements Message ----------------------------------------------

  inline NetRequest* New() const PROTOBUF_FINAL { return New(NULL); }

  NetRequest* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const NetRequest& from);
  void MergeFrom(const NetRequest& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(NetRequest* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // string netType = 1;
  void clear_nettype();
  static const int kNetTypeFieldNumber = 1;
  const ::std::string& nettype() const;
  void set_nettype(const ::std::string& value);
  #if LANG_CXX11
  void set_nettype(::std::string&& value);
  #endif
  void set_nettype(const char* value);
  void set_nettype(const char* value, size_t size);
  ::std::string* mutable_nettype();
  ::std::string* release_nettype();
  void set_allocated_nettype(::std::string* nettype);

  // bytes img = 2;
  void clear_img();
  static const int kImgFieldNumber = 2;
  const ::std::string& img() const;
  void set_img(const ::std::string& value);
  #if LANG_CXX11
  void set_img(::std::string&& value);
  #endif
  void set_img(const char* value);
  void set_img(const void* value, size_t size);
  ::std::string* mutable_img();
  ::std::string* release_img();
  void set_allocated_img(::std::string* img);

  // @@protoc_insertion_point(class_scope:netVlad.NetRequest)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::ArenaStringPtr nettype_;
  ::google::protobuf::internal::ArenaStringPtr img_;
  mutable int _cached_size_;
  friend struct protobuf_netVlad_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class NetResponse : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:netVlad.NetResponse) */ {
 public:
  NetResponse();
  virtual ~NetResponse();

  NetResponse(const NetResponse& from);

  inline NetResponse& operator=(const NetResponse& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const NetResponse& default_instance();

  static inline const NetResponse* internal_default_instance() {
    return reinterpret_cast<const NetResponse*>(
               &_NetResponse_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    1;

  void Swap(NetResponse* other);

  // implements Message ----------------------------------------------

  inline NetResponse* New() const PROTOBUF_FINAL { return New(NULL); }

  NetResponse* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const NetResponse& from);
  void MergeFrom(const NetResponse& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(NetResponse* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated float vec = 2;
  int vec_size() const;
  void clear_vec();
  static const int kVecFieldNumber = 2;
  float vec(int index) const;
  void set_vec(int index, float value);
  void add_vec(float value);
  const ::google::protobuf::RepeatedField< float >&
      vec() const;
  ::google::protobuf::RepeatedField< float >*
      mutable_vec();

  // string flag = 1;
  void clear_flag();
  static const int kFlagFieldNumber = 1;
  const ::std::string& flag() const;
  void set_flag(const ::std::string& value);
  #if LANG_CXX11
  void set_flag(::std::string&& value);
  #endif
  void set_flag(const char* value);
  void set_flag(const char* value, size_t size);
  ::std::string* mutable_flag();
  ::std::string* release_flag();
  void set_allocated_flag(::std::string* flag);

  // @@protoc_insertion_point(class_scope:netVlad.NetResponse)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedField< float > vec_;
  mutable int _vec_cached_byte_size_;
  ::google::protobuf::internal::ArenaStringPtr flag_;
  mutable int _cached_size_;
  friend struct protobuf_netVlad_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// NetRequest

// string netType = 1;
inline void NetRequest::clear_nettype() {
  nettype_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline const ::std::string& NetRequest::nettype() const {
  // @@protoc_insertion_point(field_get:netVlad.NetRequest.netType)
  return nettype_.GetNoArena();
}
inline void NetRequest::set_nettype(const ::std::string& value) {
  
  nettype_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:netVlad.NetRequest.netType)
}
#if LANG_CXX11
inline void NetRequest::set_nettype(::std::string&& value) {
  
  nettype_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:netVlad.NetRequest.netType)
}
#endif
inline void NetRequest::set_nettype(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  
  nettype_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:netVlad.NetRequest.netType)
}
inline void NetRequest::set_nettype(const char* value, size_t size) {
  
  nettype_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:netVlad.NetRequest.netType)
}
inline ::std::string* NetRequest::mutable_nettype() {
  
  // @@protoc_insertion_point(field_mutable:netVlad.NetRequest.netType)
  return nettype_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* NetRequest::release_nettype() {
  // @@protoc_insertion_point(field_release:netVlad.NetRequest.netType)
  
  return nettype_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void NetRequest::set_allocated_nettype(::std::string* nettype) {
  if (nettype != NULL) {
    
  } else {
    
  }
  nettype_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), nettype);
  // @@protoc_insertion_point(field_set_allocated:netVlad.NetRequest.netType)
}

// bytes img = 2;
inline void NetRequest::clear_img() {
  img_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline const ::std::string& NetRequest::img() const {
  // @@protoc_insertion_point(field_get:netVlad.NetRequest.img)
  return img_.GetNoArena();
}
inline void NetRequest::set_img(const ::std::string& value) {
  
  img_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:netVlad.NetRequest.img)
}
#if LANG_CXX11
inline void NetRequest::set_img(::std::string&& value) {
  
  img_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:netVlad.NetRequest.img)
}
#endif
inline void NetRequest::set_img(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  
  img_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:netVlad.NetRequest.img)
}
inline void NetRequest::set_img(const void* value, size_t size) {
  
  img_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:netVlad.NetRequest.img)
}
inline ::std::string* NetRequest::mutable_img() {
  
  // @@protoc_insertion_point(field_mutable:netVlad.NetRequest.img)
  return img_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* NetRequest::release_img() {
  // @@protoc_insertion_point(field_release:netVlad.NetRequest.img)
  
  return img_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void NetRequest::set_allocated_img(::std::string* img) {
  if (img != NULL) {
    
  } else {
    
  }
  img_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), img);
  // @@protoc_insertion_point(field_set_allocated:netVlad.NetRequest.img)
}

// -------------------------------------------------------------------

// NetResponse

// string flag = 1;
inline void NetResponse::clear_flag() {
  flag_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline const ::std::string& NetResponse::flag() const {
  // @@protoc_insertion_point(field_get:netVlad.NetResponse.flag)
  return flag_.GetNoArena();
}
inline void NetResponse::set_flag(const ::std::string& value) {
  
  flag_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:netVlad.NetResponse.flag)
}
#if LANG_CXX11
inline void NetResponse::set_flag(::std::string&& value) {
  
  flag_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:netVlad.NetResponse.flag)
}
#endif
inline void NetResponse::set_flag(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  
  flag_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:netVlad.NetResponse.flag)
}
inline void NetResponse::set_flag(const char* value, size_t size) {
  
  flag_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:netVlad.NetResponse.flag)
}
inline ::std::string* NetResponse::mutable_flag() {
  
  // @@protoc_insertion_point(field_mutable:netVlad.NetResponse.flag)
  return flag_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* NetResponse::release_flag() {
  // @@protoc_insertion_point(field_release:netVlad.NetResponse.flag)
  
  return flag_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void NetResponse::set_allocated_flag(::std::string* flag) {
  if (flag != NULL) {
    
  } else {
    
  }
  flag_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), flag);
  // @@protoc_insertion_point(field_set_allocated:netVlad.NetResponse.flag)
}

// repeated float vec = 2;
inline int NetResponse::vec_size() const {
  return vec_.size();
}
inline void NetResponse::clear_vec() {
  vec_.Clear();
}
inline float NetResponse::vec(int index) const {
  // @@protoc_insertion_point(field_get:netVlad.NetResponse.vec)
  return vec_.Get(index);
}
inline void NetResponse::set_vec(int index, float value) {
  vec_.Set(index, value);
  // @@protoc_insertion_point(field_set:netVlad.NetResponse.vec)
}
inline void NetResponse::add_vec(float value) {
  vec_.Add(value);
  // @@protoc_insertion_point(field_add:netVlad.NetResponse.vec)
}
inline const ::google::protobuf::RepeatedField< float >&
NetResponse::vec() const {
  // @@protoc_insertion_point(field_list:netVlad.NetResponse.vec)
  return vec_;
}
inline ::google::protobuf::RepeatedField< float >*
NetResponse::mutable_vec() {
  // @@protoc_insertion_point(field_mutable_list:netVlad.NetResponse.vec)
  return &vec_;
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)


}  // namespace netVlad

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_netVlad_2eproto__INCLUDED
