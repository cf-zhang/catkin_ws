#ifndef EXTENDEDTABLEWIDGET_H
#define EXTENDEDTABLEWIDGET_H

#include <QTableView>
#include <QSet>
#include <QDropEvent>
#include <QDragMoveEvent>

class QMenu;
class FilterTableHeader;
namespace sqlb { class ObjectIdentifier; }

class ExtendedTableWidget : public QTableView
{
    Q_OBJECT

public:
    explicit ExtendedTableWidget(QWidget* parent = NULL);

    FilterTableHeader* filterHeader() { return m_tableHeader; }

public:
    QSet<int> selectedCols();
    int numVisibleRows();

public Q_SLOTS:
    void reloadSettings();

//signals:
Q_SIGNALS:
    void foreignKeyClicked(const sqlb::ObjectIdentifier& table, const QString& column, const QByteArray& value);
    void switchTable(bool next);    // 'next' parameter is set to true if next table should be selected and to false if previous table should be selected

private:
    void copy(const bool withHeaders = false);
    void paste();
    void useAsFilter();

    typedef QList<QByteArray> QByteArrayList;
    QList<QByteArrayList> m_buffer;

private Q_SLOTS:
    void vscrollbarChanged(int value);
    void cellClicked(const QModelIndex& index);

protected:
    virtual void keyPressEvent(QKeyEvent* event);
    virtual void updateGeometries();
    virtual void dragEnterEvent(QDragEnterEvent* event);
    virtual void dragMoveEvent(QDragMoveEvent* event);
    virtual void dropEvent(QDropEvent* event);

    FilterTableHeader* m_tableHeader;
    QMenu* m_contextMenu;
};

#endif
