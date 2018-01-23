#ifndef FILTERTABLEHEADER_H
#define FILTERTABLEHEADER_H

#include <QHeaderView>
#include <QList>

class QLineEdit;
class QTableView;
class FilterLineEdit;

class FilterTableHeader : public QHeaderView
{
    Q_OBJECT

public:
    explicit FilterTableHeader(QTableView* parent = NULL);
    virtual QSize sizeHint() const;
    
public Q_SLOTS:
    void generateFilters(int number, bool showFirst = false);
    void adjustPositions();
    void clearFilters();
    void setFilter(int column, const QString& value);

Q_SIGNALS:
    void filterChanged(int column, QString value);

protected:
    virtual void updateGeometries();

private Q_SLOTS:
    void inputChanged(const QString& new_value);

private:
    QList<FilterLineEdit*> filterWidgets;
};

#endif
