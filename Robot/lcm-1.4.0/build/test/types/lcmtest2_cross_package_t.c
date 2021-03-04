// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "lcmtest2_cross_package_t.h"

static int __lcmtest2_cross_package_t_hash_computed;
static uint64_t __lcmtest2_cross_package_t_hash;

uint64_t __lcmtest2_cross_package_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __lcmtest2_cross_package_t_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = __lcmtest2_cross_package_t_get_hash;
    (void) cp;

    uint64_t hash = (uint64_t)0xbbd3dd8a23ec1955LL
         + __lcmtest_primitives_t_hash_recursive(&cp)
         + __lcmtest2_another_type_t_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __lcmtest2_cross_package_t_get_hash(void)
{
    if (!__lcmtest2_cross_package_t_hash_computed) {
        __lcmtest2_cross_package_t_hash = (int64_t)__lcmtest2_cross_package_t_hash_recursive(NULL);
        __lcmtest2_cross_package_t_hash_computed = 1;
    }

    return __lcmtest2_cross_package_t_hash;
}

int __lcmtest2_cross_package_t_encode_array(void *buf, int offset, int maxlen, const lcmtest2_cross_package_t *p, int elements)
{
    int pos = 0, element;
    int thislen;

    for (element = 0; element < elements; element++) {

        thislen = __lcmtest_primitives_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].primitives), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __lcmtest2_another_type_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].another), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int lcmtest2_cross_package_t_encode(void *buf, int offset, int maxlen, const lcmtest2_cross_package_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __lcmtest2_cross_package_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __lcmtest2_cross_package_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __lcmtest2_cross_package_t_encoded_array_size(const lcmtest2_cross_package_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __lcmtest_primitives_t_encoded_array_size(&(p[element].primitives), 1);

        size += __lcmtest2_another_type_t_encoded_array_size(&(p[element].another), 1);

    }
    return size;
}

int lcmtest2_cross_package_t_encoded_size(const lcmtest2_cross_package_t *p)
{
    return 8 + __lcmtest2_cross_package_t_encoded_array_size(p, 1);
}

int __lcmtest2_cross_package_t_decode_array(const void *buf, int offset, int maxlen, lcmtest2_cross_package_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __lcmtest_primitives_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].primitives), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __lcmtest2_another_type_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].another), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __lcmtest2_cross_package_t_decode_array_cleanup(lcmtest2_cross_package_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __lcmtest_primitives_t_decode_array_cleanup(&(p[element].primitives), 1);

        __lcmtest2_another_type_t_decode_array_cleanup(&(p[element].another), 1);

    }
    return 0;
}

int lcmtest2_cross_package_t_decode(const void *buf, int offset, int maxlen, lcmtest2_cross_package_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __lcmtest2_cross_package_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __lcmtest2_cross_package_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int lcmtest2_cross_package_t_decode_cleanup(lcmtest2_cross_package_t *p)
{
    return __lcmtest2_cross_package_t_decode_array_cleanup(p, 1);
}

int __lcmtest2_cross_package_t_clone_array(const lcmtest2_cross_package_t *p, lcmtest2_cross_package_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __lcmtest_primitives_t_clone_array(&(p[element].primitives), &(q[element].primitives), 1);

        __lcmtest2_another_type_t_clone_array(&(p[element].another), &(q[element].another), 1);

    }
    return 0;
}

lcmtest2_cross_package_t *lcmtest2_cross_package_t_copy(const lcmtest2_cross_package_t *p)
{
    lcmtest2_cross_package_t *q = (lcmtest2_cross_package_t*) malloc(sizeof(lcmtest2_cross_package_t));
    __lcmtest2_cross_package_t_clone_array(p, q, 1);
    return q;
}

void lcmtest2_cross_package_t_destroy(lcmtest2_cross_package_t *p)
{
    __lcmtest2_cross_package_t_decode_array_cleanup(p, 1);
    free(p);
}

int lcmtest2_cross_package_t_publish(lcm_t *lc, const char *channel, const lcmtest2_cross_package_t *p)
{
      int max_data_size = lcmtest2_cross_package_t_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = lcmtest2_cross_package_t_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _lcmtest2_cross_package_t_subscription_t {
    lcmtest2_cross_package_t_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void lcmtest2_cross_package_t_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    lcmtest2_cross_package_t p;
    memset(&p, 0, sizeof(lcmtest2_cross_package_t));
    status = lcmtest2_cross_package_t_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding lcmtest2_cross_package_t!!!\n", status);
        return;
    }

    lcmtest2_cross_package_t_subscription_t *h = (lcmtest2_cross_package_t_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    lcmtest2_cross_package_t_decode_cleanup (&p);
}

lcmtest2_cross_package_t_subscription_t* lcmtest2_cross_package_t_subscribe (lcm_t *lcm,
                    const char *channel,
                    lcmtest2_cross_package_t_handler_t f, void *userdata)
{
    lcmtest2_cross_package_t_subscription_t *n = (lcmtest2_cross_package_t_subscription_t*)
                       malloc(sizeof(lcmtest2_cross_package_t_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 lcmtest2_cross_package_t_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg lcmtest2_cross_package_t LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int lcmtest2_cross_package_t_subscription_set_queue_capacity (lcmtest2_cross_package_t_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int lcmtest2_cross_package_t_unsubscribe(lcm_t *lcm, lcmtest2_cross_package_t_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe lcmtest2_cross_package_t_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}

