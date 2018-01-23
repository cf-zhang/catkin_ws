#include "rsa.h"

TRsa::TRsa()
{
    pub_key = RSA_new();
    pri_key = RSA_new();
    rsa_generate_key();
}

TRsa::TRsa(const string &pub_path, const string pri_path)
{
    pub_key = RSA_new();
    pri_key = RSA_new();
    rsa_read_public_key(pub_path);
    rsa_read_private_key(pri_path);
}

TRsa::~TRsa()
{
    if (pub_key != NULL)
    {
        RSA_free(pub_key);
        pub_key = NULL;
    }

    if (pri_key)
    {
        RSA_free(pri_key);
        pri_key = NULL;
    }
}

int TRsa::rsa_generate_key(const string &path, const string &password)
{
    RSA *key = NULL;
    key = RSA_generate_key(RSA_KEY_LENGTH, RSA_F4, NULL, NULL);

    FILE *pri_fp = fopen(path.c_str(), "w");
    if (!PEM_write_RSAPrivateKey(pri_fp, key, NULL, NULL, 0, 0, NULL))
    {
        return ERROR_WRITE_PRIVATE_KEY;
    }

    string temp = path;
    temp += ".pub";

    FILE *pub_fp = fopen(temp.c_str(), "w");
    if (!PEM_write_RSAPublicKey(pub_fp, key))
    {
        return ERROR_WRITE_PUBLIC_KEY;
    }
    fclose(pri_fp);
    fclose(pub_fp);
    return 0;
}

int TRsa::rsa_generate_key()
{
    RSA *key = NULL;
    key = RSA_generate_key(RSA_KEY_LENGTH, RSA_F4, NULL, NULL);
    pub_key = RSAPublicKey_dup(key);
    pri_key = RSAPrivateKey_dup(key);
}

int TRsa::rsa_read_public_key(const string &path)
{
    FILE *pub_fp;
    if (pub_key == NULL)
    {
        return ERROR_PUBLIC_NOT_INITIAL;
    }

    pub_fp = fopen(path.c_str(), "rb");
    if((pub_key = PEM_read_RSAPublicKey(pub_fp, &pub_key, NULL, NULL)) == NULL)
    {
        ERR_print_errors_fp(stdout);
    }
    fclose(pub_fp);
    return 0;
}

int TRsa::rsa_read_private_key(const string &path)
{
    FILE *pri_fp;
    if (pri_key == NULL)
    {
        return ERROR_PRIVATE_NOT_INITIAL;
    }

    pri_fp = fopen(path.c_str(), "rb");
    PEM_read_RSAPrivateKey(pri_fp, &pri_key, NULL, NULL);
    fclose(pri_fp);
    return 0;
}

int TRsa::rsa_set_public_key(const unsigned char *module, int length)
{
    //怀疑openssll在pubkey存储了中间状态！不释放公钥重新设置了公钥之后加密服务器解密失败
    if (pub_key != NULL)
    {
        RSA_free(pub_key);
        pub_key = NULL;
    }
    pub_key = RSA_new();
    pub_key->e = BN_bin2bn(EXPONENT_HEX, 3, pub_key->e);
    pub_key->n = BN_bin2bn(module, length, pub_key->n);
    return 0;
}

int TRsa::get_rsa_public_key(char *key, int length)
{
    BN_bn2bin(pub_key->n, (unsigned char*)key);
    return 0;
#if 0
    char *p;
    if( p = BN_bn2bin(pub_key->n, key))
    {
        memcpy(key, p, length);
        OPENSSL_free (p);
        return 0;
    }
    return -1;
#endif // 0
}

int TRsa::private_key_encrypt(const string &src, string &dest)
{
    int ret;
    int out_len = RSA_size(pri_key);
    unsigned char *out = (unsigned char *)malloc(out_len);
    if(out == NULL)
    {
        return ERROR_MALLOC;
    }
    memset(out, 0, out_len);
    ret = RSA_private_encrypt(src.length(), (const unsigned char *)src.c_str(), out, pri_key, RSA_PKCS1_PADDING);
    dest.replace(0, 0, (char *)out, strlen((char *)out));
    free(out);
    return ret;
}

int TRsa::public_key_encrypt(const string &src, string &dest)
{
    int ret;
    int out_len = RSA_size(pub_key);
    unsigned char *out = (unsigned char *)malloc(out_len);
    if(out == NULL)
    {
        return ERROR_MALLOC;
    }
    memset(out, 0, out_len);
    ret = RSA_public_encrypt(src.length(), (const unsigned char *)src.c_str(), out, pub_key, RSA_PKCS1_PADDING);
    dest.replace(0, 0, (char *)out, out_len);
    free(out);
    return ret;
}

int TRsa::private_key_decrypt(const string &src, string &dest)
{
    int ret;
    int out_len = RSA_size(pri_key);
    unsigned char *out = (unsigned char *)malloc(out_len);
    if(out == NULL)
    {
        return ERROR_MALLOC;
    }
    memset(out, 0, out_len);
    ret = RSA_private_decrypt(src.length(), (const unsigned char *)src.c_str(), out, pri_key, RSA_PKCS1_PADDING);
    dest.replace(0, 0, (char *)out, strlen((char *)out));

    free(out);
    return ret;
}

int TRsa::private_key_decrypt(char *src, int src_len, char *dest, int dest_len)
{
    int ret;
    int out_len = RSA_size(pri_key);
    ret = RSA_private_decrypt(src_len, (const unsigned char *)src, (unsigned char *)dest, pri_key, RSA_PKCS1_PADDING);
    return ret;
}

int TRsa::public_key_decrypt(const string &src, string &dest)
{
    int ret;
    int out_len = RSA_size(pub_key);
    unsigned char *out = (unsigned char *)malloc(out_len);
    if(out == NULL)
    {
        return ERROR_MALLOC;
    }
    memset(out, 0, out_len);
    ret = RSA_public_decrypt(src.length(), (const unsigned char *)src.c_str(), out, pub_key, RSA_PKCS1_PADDING);
    dest.replace(0, 0, (char *)out, strlen((char *)out));

    free(out);
    return ret;
}
