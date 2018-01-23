#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QTimer>
#include <QMenu>
#include <QLabel>
#include <QInputDialog>
#include <QString>
#include <QMessageBox>
#include <QLineEdit>
#include <QComboBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <QToolButton>
#include <QApplication>
#include <QSpacerItem>
#include <QHeaderView>
#include <ros/ros.h>
#include "ExtendedTableWidget.h"
#include "FilterTableHeader.h"
#include "sqlitedb.h"
#include "DbStructureModel.h"
#include "sqlitetablemodel.h"
#include "Settings.h"
#include "sqlite.h"
#include "FileDialog.h"
namespace log_panel
{
class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();
protected:
    void resizeEvent(QResizeEvent *event);
private:
    ros::NodeHandle nh_;
    ros::ServiceClient client;
    QTimer* update_timer_;
    const QString iconsPath;

    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout;
    QLabel      *label;
    QComboBox *comboBrowseTable;
    QToolButton *buttonRefresh;
    QToolButton *buttonClearFilters;
    QSpacerItem *horizontalSpacer;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *buttonNewRecord;
    QPushButton *buttonDeleteRecord;
    QHBoxLayout *horizontalLayout_2;
    QToolButton *buttonBegin;
    QToolButton *buttonPrevious;
    QLabel *labelRecordset;
    QToolButton *buttonNext;
    QToolButton *buttonEnd;
    QPushButton *buttonGoto;
    QLineEdit *editGoto;
    ExtendedTableWidget *dataTable;
    QIntValidator* gotoValidator;
    SqliteTableModel* m_browseTableModel;
    DbStructureModel *dbStructureModel;

    DBBrowserDB db;
    QAction *actionShowRowidColumn;
    QAction *actionUnlockViewEditing;
    QAction *actionBrowseTableEditDisplayFormat;
    QMenu* popupBrowseDataHeaderMenu;
    void initLayout();
    void initVariable();
    void initConnect();

    sqlb::ObjectIdentifier currentlyBrowsedTableName() const;
    void enableEditing(bool enable_edit, bool enable_insertdelete);
    
    void clearTableBrowser();
    void populateTable();
    void setCurrentFile(const QString &fileName);
    bool fileClose();
    bool fileOpen(const QString& fileName = QString(), bool dontAddToRecentFiles = false, bool readOnly = false);
    void activateFields(bool enable);
private Q_SLOTS:
    void onUpdate();
    void requestCollation(const QString& name, int eTextRep);
    void on_actionShowAllColumns_triggered();
    void hideColumns(int column = -1, bool hide = true);
    void unlockViewEditing(bool unlock, QString pk=QString());
    void populateStructure();
    void updateFilter(int column, const QString& value);
    void setRecordsetLabel();
    void navigateGoto();
    void navigatePrevious();
    void navigateNext();
    void navigateBegin();
    void navigateEnd();    
    void selectTableLine( int lineToSelect );
    void selectTableLines(int firstLine, int count);
    void dbState( bool dirty );
    void refresh();
    void addRecord();
    void deleteRecord();
    void on_buttonClearFilters_clicked();
    void showRowidColumn(bool show);
    void editDataColumnDisplayFormat();
    void browseTableHeaderClicked(int logicalindex);
    void dataTableSelectionChanged(const QModelIndex& index);
};
}
#endif // WIDGET_H
