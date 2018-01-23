#ifndef LOGINDLG_H
#define LOGINDLG_H
#include <QDialog>

#include <QVariant>
#include <QAction>
#include <QApplication>
#include <QButtonGroup>
#include <QDialog>
#include <QGridLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QSpacerItem>
#include <QTabWidget>
#include <QWidget>
#include <QHBoxLayout>
#include <QtSql>
#include "rsa.h"
class LoginDlg : public QDialog {
    Q_OBJECT
public:
    LoginDlg(QWidget *parent = 0);
    ~LoginDlg();
    bool appendAccount(const std::string &userName, const std::string &passWord);
protected:
//    void changeEvent(QEvent *e);

private:
    QHBoxLayout  *h_layout;
    QTabWidget   *loginTabWidget;
    QWidget      *accountLogin;
    QWidget      *accountRegister;
    QWidget      *faceRecogLogin;
    QGridLayout  *gridLayout;
    QGridLayout  *gridLayoutR;
    QLabel       *userNameLabel;
    QLabel       *passWordLabel;
    QLineEdit    *userNameEdit;
    QLineEdit    *passWordEdit;
    QRadioButton *rememberMe;
    QPushButton *registerButton;
    QPushButton  *signInButton;
    QPushButton  *quitButton;

    QLabel *labelRegName;
    QLabel *labelRegPwd;
    QLabel *labelRegRepeatPwd;
    QPushButton  *pushButtonQuit;
    QPushButton  *pushButtonRegister;
    QLineEdit       *passwdReapeatEdit;
    QLineEdit       *passwdRegEdit;
    QLineEdit       *userNameRegEdit;
    QSpacerItem    *horizontalSpacer;

    enum SignInMode{ACCOUNT, FACE} signInMode;
    bool isSignInSucceed;
    string rememberMeFile;
    QSqlDatabase db;
    QSqlQuery query;
    std::map<std::string, std::string> loginMap;
    TRsa *rsa;

private Q_SLOTS:
    void signInCallBack();
    void showRegisterPanel();
    void registerCallBack();
private:
    LoginDlg & operator=(const LoginDlg &){};
    LoginDlg(const LoginDlg &){};  
    void createRegisterPanel();
    void createLoginPanel();
    void setupAccountWidget();
    void loadRememberedAccount();
    void saveCurrentAccount(const std::string &userName, const std::string &passwd);
    void deleteRememberMeFile(const std::string &userName);
    bool checkPassword(const QString &userName, const QString &passWord);
};
#endif // LOGINDLG_H
