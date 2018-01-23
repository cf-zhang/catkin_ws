#include "logindlg.h"
#include "base64.h"
#include <rpc/des_crypt.h>
#include <fstream>
#include <iostream>
#include <string>
#include "QMessageBox"
#include "QTextCodec"

void LoginDlg::setupAccountWidget()
{
    createLoginPanel();
    createRegisterPanel();
}

LoginDlg::LoginDlg(QWidget *parent) :
    QDialog(parent)
{
    this->resize(360,240);
    h_layout = new QHBoxLayout;
    loginTabWidget = new QTabWidget;

    
    //account login mode
    setupAccountWidget();
    QHBoxLayout *layout = new QHBoxLayout(this);    //水平布局
    layout->addWidget(accountLogin);
    layout->addWidget(accountRegister);
    layout->setSizeConstraint(QLayout::SetFixedSize);   //设定其为固定的大小
    layout->setSpacing(10);

    QWidget *login = new QWidget();
    login->setLayout(layout);
    loginTabWidget->addTab(login, "Account");

    //face login mode
    faceRecogLogin = new QWidget();
    loginTabWidget->addTab(faceRecogLogin, "Face");

    h_layout->addWidget(loginTabWidget);
    setWindowFlags(Qt::FramelessWindowHint | Qt::WindowCloseButtonHint);
    setLayout(h_layout);

    rsa = new TRsa("/home/cfzhang/catkin_ws/src/rviz/rsakey.pub",
                   "/home/cfzhang/catkin_ws/src/rviz/rsakey");
    //read the rememberMe file for editLine;
    rememberMeFile = "/home/cfzhang/catkin_ws/src/rviz/remebered";
    loadRememberedAccount();

    signInMode = (SignInMode)loginTabWidget->currentIndex();
    isSignInSucceed = false;
    //set up the database
    db = QSqlDatabase::addDatabase("QSQLITE");
    db.setDatabaseName("/home/cfzhang/catkin_ws/src/rviz/my.db");
    if (!db.open())
    {
        qDebug()<<"open database failed ---"<<db.lastError().text()<<"/n";
    }
    query = QSqlQuery(db);
}

void LoginDlg::createLoginPanel()
{
    accountLogin = new QWidget();
    gridLayout = new QGridLayout();
    QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Fixed);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);

    //username label setup
    userNameLabel = new QLabel("UserName : ",accountLogin);
    sizePolicy1.setHeightForWidth(userNameLabel->sizePolicy().hasHeightForWidth());
    userNameLabel->setSizePolicy(sizePolicy1);
    QFont font;
    font.setPointSize(17);
    userNameLabel->setFont(font);
    gridLayout->addWidget(userNameLabel, 0, 1, 1, 1);
    //username lineEdit setup
    userNameEdit = new QLineEdit(accountLogin);
    gridLayout->addWidget(userNameEdit, 0, 2, 1, 2);

    //passWord label setup
    passWordLabel = new QLabel("PassWord : ",accountLogin);
    sizePolicy1.setHeightForWidth(passWordLabel->sizePolicy().hasHeightForWidth());
    passWordLabel->setSizePolicy(sizePolicy1);
    passWordLabel->setFont(font);
    gridLayout->addWidget(passWordLabel, 1, 1, 1, 1);
    passWordEdit = new QLineEdit(accountLogin);
    passWordEdit->setEchoMode(QLineEdit::Password);
    gridLayout->addWidget(passWordEdit, 1, 2, 1, 2);

    //remember me radiobutton setup
    rememberMe = new QRadioButton("Remember Me",accountLogin);
    sizePolicy1.setHeightForWidth(rememberMe->sizePolicy().hasHeightForWidth());
    rememberMe->setSizePolicy(sizePolicy1);
    rememberMe->setChecked(true);
    gridLayout->addWidget(rememberMe, 2, 1, 1, 1);
   
    //
    signInButton = new QPushButton("Sign In", accountLogin);
    QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Fixed);
    sizePolicy2.setHorizontalStretch(40);
    sizePolicy2.setVerticalStretch(20);
    sizePolicy2.setHeightForWidth(signInButton->sizePolicy().hasHeightForWidth());
    signInButton->setSizePolicy(sizePolicy2);
    signInButton->setMinimumSize(QSize(30, 20));
    QFont font1;
    font1.setPointSize(32);
    signInButton->setFont(font1);
    gridLayout->addWidget(signInButton, 3, 1, 1, 2);

    quitButton = new QPushButton("Quit",accountLogin);
    sizePolicy2.setHeightForWidth(quitButton->sizePolicy().hasHeightForWidth());
    quitButton->setSizePolicy(sizePolicy2);
    quitButton->setMinimumSize(QSize(30, 20));
    font1.setPointSize(24);
    quitButton->setFont(font1);
    gridLayout->addWidget(quitButton, 3, 3, 1, 1);

 
    registerButton = new QPushButton("Register",accountLogin);
    sizePolicy2.setHeightForWidth(registerButton->sizePolicy().hasHeightForWidth());
    registerButton->setSizePolicy(sizePolicy2);
    registerButton->setMinimumSize(QSize(30, 20));
    // font1.setPointSize(24);
    // registerButton->setFont(font1);
    gridLayout->addWidget(registerButton, 2, 3, 1, 1);

    accountLogin->setLayout(gridLayout);
        userNameEdit->setFocus();

    connect(quitButton, SIGNAL(pressed()), this, SLOT(close()));
    connect(signInButton, SIGNAL(pressed()), this, SLOT(signInCallBack()));
    connect(registerButton, SIGNAL(pressed()), this, SLOT(showRegisterPanel()));

}

void LoginDlg::createRegisterPanel()
{
    accountRegister = new QWidget(this);

    gridLayoutR = new QGridLayout(accountRegister);
    gridLayoutR->setSpacing(6);
    gridLayoutR->setContentsMargins(11, 11, 11, 11);
    gridLayoutR->setContentsMargins(0, 0, 0, 0);

    labelRegName = new QLabel(accountRegister);
    labelRegName->setObjectName(QStringLiteral("label"));

    gridLayoutR->addWidget(labelRegName, 0, 0, 1, 1);

    userNameRegEdit = new QLineEdit(accountRegister);
    userNameRegEdit->setObjectName(QStringLiteral("lineEdit"));

    gridLayoutR->addWidget(userNameRegEdit, 0, 1, 1, 2);

    labelRegPwd = new QLabel(accountRegister);
    labelRegPwd->setObjectName(QStringLiteral("label_4"));

    gridLayoutR->addWidget(labelRegPwd, 1, 0, 1, 1);

    passwdRegEdit = new QLineEdit(accountRegister);
    passwdRegEdit->setObjectName(QStringLiteral("lineEdit_4"));

    gridLayoutR->addWidget(passwdRegEdit, 1, 1, 1, 2);

    labelRegRepeatPwd = new QLabel(accountRegister);
    labelRegRepeatPwd->setObjectName(QStringLiteral("label_5"));

    gridLayoutR->addWidget(labelRegRepeatPwd, 2, 0, 1, 1);

    passwdReapeatEdit = new QLineEdit(accountRegister);
    passwdReapeatEdit->setObjectName(QStringLiteral("lineEdit_5"));

    gridLayoutR->addWidget(passwdReapeatEdit, 2, 1, 1, 2);

    // pushButtonQuit = new QPushButton(accountRegister);
    // pushButtonQuit->setObjectName(QStringLiteral("pushButton_2"));

    // gridLayoutR->addWidget(pushButtonQuit, 3, 0, 1, 1);

    horizontalSpacer = new QSpacerItem(63, 38, QSizePolicy::Expanding, QSizePolicy::Minimum);

    gridLayoutR->addItem(horizontalSpacer, 3, 1, 1, 1);

    pushButtonRegister = new QPushButton(accountRegister);
    pushButtonRegister->setObjectName(QStringLiteral("pushButton"));

    gridLayoutR->addWidget(pushButtonRegister, 3, 2, 1, 1);

    // Widget->setWindowTitle(QApplication::translate("Widget", "Widget", nullptr));
    labelRegName->setText(QApplication::translate("Widget", "userName", NULL));
    labelRegPwd->setText(QApplication::translate("Widget", "passwd", NULL));
    labelRegRepeatPwd->setText(QApplication::translate("Widget", "repeat passwd", NULL));
    // pushButtonQuit->setText(QApplication::translate("Widget", "Quit", NULL));
    pushButtonRegister->setText(QApplication::translate("Widget", "Register", NULL));

    connect(pushButtonRegister, SIGNAL(pressed()), this, SLOT(registerCallBack()));
    accountRegister->hide(); 

}
LoginDlg::~LoginDlg(){
    QSqlDatabase::removeDatabase("QSQLITE");
    delete userNameLabel;
    delete passWordLabel;
    delete userNameEdit;
    delete passWordEdit;
    delete rememberMe;
    delete signInButton;
    delete quitButton;
    delete gridLayout;
    delete faceRecogLogin;
    delete accountLogin;
    delete loginTabWidget;
    delete h_layout;
};


bool LoginDlg::appendAccount(const std::string &userName, const std::string &passWord)
{
    std::string dest;
    rsa->public_key_encrypt(passWord, dest);

    std::string encoded = base64_encode(dest.c_str(), dest.length());

    bool ok = query.exec("CREATE TABLE IF NOT EXISTS  rviz_user ("
                                       "username VARCHAR(32) PRIMARY KEY  NOT NULL,"
                                       "passwd VARCHAR(1024)  NULL)");
    if (!ok)
    {
        qDebug()<<"ceate table partition failed/n";
        return ok;
    }

    query.prepare("INSERT INTO rviz_user (username, passwd) VALUES (:username, :passwd)");
    query.bindValue(":username", QString::fromStdString(userName));
    query.bindValue(":passwd", QString::fromStdString(encoded));
    ok = query.exec();
    return ok;
}


bool LoginDlg::checkPassword(const QString &userName, const QString &passWord)
{
    bool ret = false;
    QString sel = "SELECT username, passwd FROM rviz_user where username=\'"+userName+"\'";
    query.exec(sel);
    if(query.next())
    {
        std::string dest;
        std::string src = base64_decode(query.value(1).toString().toStdString());

        rsa->private_key_decrypt(src, dest);

        ret = passWord.toStdString() == dest ? true : false;
        if(ret)
        {
            if(rememberMe->isChecked())
            {
                saveCurrentAccount(userName.toStdString(), src);
            }
            else
            {//if the account is not remembered yet,remove the file
                deleteRememberMeFile(userName.toStdString());
            }

        }
    }
    return ret;
}


void LoginDlg::loadRememberedAccount()
{
    char buffer[1024];
    std::ifstream istrm(rememberMeFile.c_str(), std::ios::binary);
    if (!istrm.is_open())
    {
      return ;
    }
    int filelen = istrm.seekg(0,std::ios_base::end).tellg();
    istrm.seekg(0,std::ios_base::beg);
    istrm.getline(buffer, sizeof(buffer));
    userNameEdit->setText(QString(buffer));

    int remainLength = filelen - istrm.seekg(0, std::ios_base::cur).tellg();
    istrm.read(buffer, remainLength);
    buffer[remainLength]='\0';
    istrm.close();
    std::string dest;
    rsa->private_key_decrypt(std::string(buffer,remainLength), dest);
    passWordEdit->setText(QString::fromStdString(dest));
}
void LoginDlg::saveCurrentAccount(const std::string &userName, const std::string &passwd)
{
    std::ofstream ostrm(rememberMeFile.c_str(), std::ios::out);
    if (!ostrm.is_open())
    {
      return ;
    }
    ostrm<<userName<<std::endl;
    ostrm<<passwd;
    ostrm.close();
}
void LoginDlg::deleteRememberMeFile(const std::string &userName)
{
    bool isRemove = false;
    char buffer[128];
    std::ifstream istrm(rememberMeFile.c_str(), std::ios::binary);
    if (!istrm.is_open())
    {
      return ;
    }

    istrm.getline(buffer, sizeof(buffer));
    if(std::string(buffer) == userName)
    {
        isRemove = true;
    }
    istrm.close();

    if(isRemove)
        remove(rememberMeFile.c_str());
}


#include <iostream>

void LoginDlg::signInCallBack()
{
    
    //distinguish the sign in mode
    signInMode = (SignInMode)loginTabWidget->currentIndex();
    if(signInMode == LoginDlg::ACCOUNT)
    {
        //get lineEdit values
        QString user = userNameEdit->text();
        QString passwd = passWordEdit->text();
        //login action
        if(checkPassword(user, passwd))
        {
            isSignInSucceed = true;
        }
        else
        {
            isSignInSucceed = false;
        }
    }
    else if(signInMode == LoginDlg::FACE)
    {//

    }
    if(isSignInSucceed)
    {
        accept();
    }
    else
    {
        // QTextCodec::setCodecForTr( QTextCodec::codecForName("GBK") );
        QMessageBox::warning(this,tr("Warnning"),tr("UserName or passwd invalid!"),QMessageBox::Yes);
        userNameEdit->clear();
        passWordEdit->clear();
        userNameEdit->setFocus();
    }

}


void LoginDlg::showRegisterPanel()
{
    if(accountRegister->isHidden())
    {
        accountRegister->show();
    }
    else
    {
        accountRegister->hide();
    }
}
void LoginDlg::registerCallBack()
{
    QString user = userNameRegEdit->text();
    QString passwd = passwdRegEdit->text();
    QString passwdrpt = passwdReapeatEdit->text();

    // ROS_INFO("add click button is pressed!");
    QMessageBox msgBox;
        // qsave = msgBox.addButton(QMessageBox::Save);
    // QPushButton *ok = 
    msgBox.addButton(QMessageBox::Ok);



    if(passwd == passwdrpt && 
       this->appendAccount(user.toStdString(), passwd.toStdString()))
    {
        //success
        msgBox.setText("register new account succeed.");
    }
    else
    {
        //fail
        msgBox.setText("register new account fail.");
    }

    msgBox.exec();
}
