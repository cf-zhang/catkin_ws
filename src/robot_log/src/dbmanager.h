#ifndef DBMANAGER_H
#define DBMANAGER_H

#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlQuery>
#include <QtSql/QSqlError>
#include <QVariant>

/* sql语句模版
判断数据库中是否存在某个表： SELECT 1 FROM sqlite_master where type = 'table' and  name = 'book';
判断表中是否存在某字段：SELECT 1 FROM sqlite_master where type = 'table' and name= 'book' and sql like '%idtest%';
建表语句：CREATE TABLE book(id VARCHAR(20) PRIMARY KEY  , name VARCHAR(100), info VARCHAR(100));
添加字段：ALTER TABLE book ADD descinfo VARCHAR(100);
创建索引：CREATE INDEX book_index1 on book(id);
添加：INSERT INTO book(id, name, info) VALUES('1111', 'Qt编程', '这是一本好书');
删除：DELETE FROM book  WHERE id = '1111';
更新：UPDATE book SET name = 'test' WHERE id = '1111';
精确搜索：SELECT * FROM book WHERE id = '111';
模糊搜索：SELECT * FROM book WHERE name like '%:strBookName%' ESCAPE '!';
*/

/*
* 数据库管理类
*/
class DbManager
{
public:
    DbManager();
    ~DbManager();
    bool createDataFile(const QString& strFileName);//创建数据库文件
    bool openDataBase(const QString& strFileName);//打开数据库文件(Sqlite,*.db文件)
    void closeDataBase(); //关闭数据库
    bool isExistTable(const QString& strTableName);//判断数据库中是否存在该表
    bool isExistField(const QString& strTableName, const QString& strFieldName);//判断表中是否含有某字段(列)
    QSqlDatabase getDataBase() const {return m_db;}
    // static void dbTest(); //测试数据库
public:
    QSqlDatabase m_db; //数据库
};

#endif // DBMANAGER_H
