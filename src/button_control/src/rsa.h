#ifndef RSA_H
#define RSA_H


#include <stdio.h>
#include <string>
#include <string.h>
#include <openssl/rsa.h>
#include <openssl/pem.h>
#include <openssl/evp.h>
#include <openssl/rand.h>
#include <openssl/err.h>

using namespace std;

#define RSA_KEY_LENGTH 2048

#define ERROR_WRITE_PUBLIC_KEY    -1
#define ERROR_WRITE_PRIVATE_KEY   -2
#define ERROR_PUBLIC_NOT_INITIAL  -3
#define ERROR_PRIVATE_NOT_INITIAL -4
#define ERROR_MALLOC              -5

const unsigned char EXPONENT_HEX[] = {0x01, 0x00, 0x01};

class TRsa
{
public:
    TRsa();
    TRsa(const string &pub_path, const string pri_path);
    ~TRsa();
    int rsa_generate_key(const string &path, const string &password = "");
    int rsa_generate_key();
    int get_rsa_public_key(char *key, int length);
    int rsa_read_public_key(const string &path);
    int rsa_read_private_key(const string &path);

    int rsa_set_public_key(const unsigned char *module, int length);
    int private_key_encrypt(const string &src, string &dest);
    int public_key_encrypt(const string &src, string &dest);

    int private_key_decrypt(const string &src, string &dest);
    int private_key_decrypt(char *src, int src_len, char *dest, int dest_len);
    int public_key_decrypt(const string &src, string &dest);

public:
    RSA *pub_key;
    RSA *pri_key;
private:
    TRsa & operator=(const TRsa &){};
    TRsa(const TRsa &){};      
};


#endif // RSA_H
