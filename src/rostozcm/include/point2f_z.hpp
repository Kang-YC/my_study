/** THIS IS AN AUTOMATICALLY GENERATED FILE.
 *  DO NOT MODIFY BY HAND!!
 *
 *  Generated by zcm-gen
 **/

#include <zcm/zcm_coretypes.h>

#ifndef __point2f_z_hpp__
#define __point2f_z_hpp__



class point2f_z
{
    public:
        float      x;

        float      y;

    public:
        /**
         * Destructs a message properly if anything inherits from it
        */
        virtual ~point2f_z() {}

        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void* buf, uint32_t offset, uint32_t maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline uint32_t getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to reqad while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void* buf, uint32_t offset, uint32_t maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "point2f_z"
         */
        inline static const char* getTypeName();

        // ZCM support functions. Users should not call these
        inline int      _encodeNoHash(void* buf, uint32_t offset, uint32_t maxlen) const;
        inline uint32_t _getEncodedSizeNoHash() const;
        inline int      _decodeNoHash(const void* buf, uint32_t offset, uint32_t maxlen);
        inline static uint64_t _computeHash(const __zcm_hash_ptr* p);
};

int point2f_z::encode(void* buf, uint32_t offset, uint32_t maxlen) const
{
    uint32_t pos = 0;
    int thislen;
    int64_t hash = (int64_t)getHash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int point2f_z::decode(const void* buf, uint32_t offset, uint32_t maxlen)
{
    uint32_t pos = 0;
    int thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

uint32_t point2f_z::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t point2f_z::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* point2f_z::getTypeName()
{
    return "point2f_z";
}

int point2f_z::_encodeNoHash(void* buf, uint32_t offset, uint32_t maxlen) const
{
    uint32_t pos = 0;
    int thislen;

    thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->x, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->y, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int point2f_z::_decodeNoHash(const void* buf, uint32_t offset, uint32_t maxlen)
{
    uint32_t pos = 0;
    int thislen;

    thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->x, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->y, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    return pos;
}

uint32_t point2f_z::_getEncodedSizeNoHash() const
{
    uint32_t enc_size = 0;
    enc_size += __float_encoded_array_size(NULL, 1);
    enc_size += __float_encoded_array_size(NULL, 1);
    return enc_size;
}

uint64_t point2f_z::_computeHash(const __zcm_hash_ptr*)
{
    uint64_t hash = (uint64_t)0x1b462e05b409d08fLL;
    return (hash<<1) + ((hash>>63)&1);
}

#endif
