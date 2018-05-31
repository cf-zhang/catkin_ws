#include "dbmanager.h"
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QDebug>

//构造函数
DbManager::DbManager()
{

}

//析构函数
DbManager::~DbManager()
{

}

//创建数据库文件
bool DbManager::createDataFile(const QString &strFileName)
{
    if(!QFile::exists(strFileName))//文件不存在，则创建
    {
        QDir fileDir = QFileInfo(strFileName).absoluteDir();
        QString strFileDir = QFileInfo(strFileName).absolutePath();
        if(!fileDir.exists()) //路径不存在，创建路径
        {
            fileDir.mkpath(strFileDir);
        }
        QFile dbFile(strFileName);
        if(!dbFile.open(QIODevice::WriteOnly))//未成功打开
        {
            dbFile.close();
            return false;
        }
        dbFile.close();
    }
    return true;
}

//打开数据库文件(Sqlite,*.db文件)
bool DbManager::openDataBase(const QString& strFileName)
{
    m_db = QSqlDatabase::addDatabase("QSQLITE");
    m_db.setDatabaseName(strFileName);
    if(m_db.open())
    {
        return true;
    }
    return false;
}

//关闭数据库
void DbManager::closeDataBase()
{
    QSqlDatabase::removeDatabase("QSQLITE");
    m_db.close();
}

//判断数据库中是否存在该表
bool DbManager::isExistTable(const QString& strTableName)
{
    QSqlQuery query;
    QString strSql = QString("SELECT 1 FROM sqlite_master where type = 'table' and  name = '%1'").arg(strTableName);
    query.exec(strSql);
    if(query.next())
    {
        int nResult = query.value(0).toInt();//有表时返回1，无表时返回null
        if(nResult)
        {
            return true;
        }
    }
    return false;
}

//判断表中是否含有某字段(列)
bool DbManager::isExistField(const QString& strTableName, const QString& strFieldName)
{
    QSqlQuery query(m_db);
    QString strSql = QString("SELECT 1 FROM sqlite_master where type = 'table'"
                             " and name= '%1' and sql like '%%2%'").arg(strTableName).arg(strFieldName);
    query.exec(strSql);
    if(query.next())
    {
        int nResult = query.value(0).toInt();//有此字段时返回1，无字段时返回null
        if(nResult)
        {
            return true;
        }
    }
    return false;
}

// void DbManager::dbTest()
// {
//     QString strDbFile = "data.db";
//     createDataFile(strDbFile); //在程序目录创建data.db文件
//     openDataBase(strDbFile);
//     if(!isExistTable("book"))
//     {
// //        DbBook::createTable();
//         qDebug() << "create book table.";
//         if(!isExistField("book","pageNumber"))
//         {
//             qDebug() <<  "pageNumber not exist";
//         }
//     }
// //    DbBook::addBook("004", "Qt program","very good book.");
// //    DbBook::addBook("005", "C++ program", "very well.");
// //    DbBook::addBook("006", "SQLite book", "sqlite,program");
// //    QStringList bookIdList = DbBook::searchByBookName("program");
// //    DbBook::updateBookName("005","C++ network");
// //    DbBook::removeBook("006");
// //    qDebug() << bookIdList;

//     closeDataBase();
// }







