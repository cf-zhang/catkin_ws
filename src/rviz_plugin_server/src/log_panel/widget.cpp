#include "widget.h"
// #include <log_panel/robotCtl.h>
#include <QScrollBar>
namespace log_panel
{
Widget::Widget(QWidget *parent) :
    QWidget(parent),
    iconsPath("/home/cfzhang/catkin_ws/src/log_panel/icons/"),
    m_browseTableModel(new SqliteTableModel(db, this, Settings::getValue("db", "prefetchsize").toInt())),
    gotoValidator(new QIntValidator(0, 0, this))
{
  initLayout();

  initVariable();

  initConnect();
  fileOpen("/home/cfzhang/catkin_ws/my.db");

  populateTable();
}



void Widget::onUpdate()
{
  ros::spinOnce();
  // ROS_INFO_STREAM("hello world.");
  if (!ros::ok())
  {
      close();
  }
}

void Widget::initConnect()
{

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  // Connect SQL logging and database state setting to main window
  connect(&db, SIGNAL(dbChanged(bool)), this, SLOT(dbState(bool)));
  connect(dataTable->verticalScrollBar(), SIGNAL(valueChanged(int)), this, SLOT(setRecordsetLabel()));
  connect(&db, SIGNAL(structureUpdated()), this, SLOT(populateStructure()));
  connect(&db, &DBBrowserDB::requestCollation, this, &Widget::requestCollation);
  connect(actionShowRowidColumn, SIGNAL(triggered(bool)), this, SLOT(showRowidColumn(bool)));
  connect(actionUnlockViewEditing, SIGNAL(toggled(bool)), this, SLOT(unlockViewEditing(bool)));
  connect(actionBrowseTableEditDisplayFormat, SIGNAL(triggered()), this, SLOT(editDataColumnDisplayFormat()));
  connect(m_browseTableModel, &SqliteTableModel::finishedFetch, this, &Widget::setRecordsetLabel);
  connect(buttonGoto, SIGNAL(clicked()), this, SLOT(navigateGoto()));
  connect(editGoto, SIGNAL(returnPressed()), this, SLOT(navigateGoto()));
  connect(buttonRefresh, SIGNAL(clicked()), this, SLOT(refresh()));
  connect(buttonNext, SIGNAL(clicked()), this, SLOT(navigateNext()));
  connect(buttonPrevious, SIGNAL(clicked()), this, SLOT(navigatePrevious()));
  connect(buttonEnd, SIGNAL(clicked()), this, SLOT(navigateEnd()));
  connect(buttonBegin, SIGNAL(clicked()), this, SLOT(navigateBegin()));
  connect(buttonClearFilters, SIGNAL(clicked()), this, SLOT(on_buttonClearFilters_clicked()));
  connect(buttonNewRecord, SIGNAL(clicked()), this, SLOT(addRecord()));
  connect(buttonDeleteRecord, SIGNAL(clicked()), this, SLOT(deleteRecord()));
  connect(dataTable->filterHeader(), SIGNAL(sectionClicked(int)), this, SLOT(browseTableHeaderClicked(int)));
  connect(dataTable->filterHeader(), SIGNAL(filterChanged(int,QString)), this, SLOT(updateFilter(int,QString)));
  connect(dataTable, &ExtendedTableWidget::switchTable, [this](bool next) {
    int index = comboBrowseTable->currentIndex();
    int num_items = comboBrowseTable->count();
    if(next)
    {
        if(++index >= num_items)
            index = 0;
    } else {
        if(--index < 0)
            index = num_items - 1;
    }
    comboBrowseTable->setCurrentIndex(index);
    populateTable();
});
}

void Widget::initVariable()
{
  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16);
  update_timer_->start();
  actionShowRowidColumn = new QAction(this);
  actionShowRowidColumn->setObjectName(QStringLiteral("actionShowRowidColumn"));
  actionShowRowidColumn->setCheckable(true);

  actionUnlockViewEditing = new QAction(this);
  actionUnlockViewEditing->setObjectName(QStringLiteral("actionUnlockViewEditing"));
  actionUnlockViewEditing->setCheckable(true);

  actionBrowseTableEditDisplayFormat = new QAction(this);
  actionBrowseTableEditDisplayFormat->setObjectName(QStringLiteral("actionBrowseTableEditDisplayFormat"));
  // Set up DB structure tab
  dbStructureModel = new DbStructureModel(db, this);

  // Set up the table combo box in the Browse Data tab
  comboBrowseTable->setModel(dbStructureModel);
  // comboBrowseTable->setColumnWidth(DbStructureModel::ColumnName, 300);
  // comboBrowseTable->setColumnHidden(DbStructureModel::ColumnObjectType, true);
  // comboBrowseTable->setColumnHidden(DbStructureModel::ColumnSchema, true);
  popupBrowseDataHeaderMenu = new QMenu(this);
  popupBrowseDataHeaderMenu->addAction(actionShowRowidColumn);
  popupBrowseDataHeaderMenu->addAction(actionUnlockViewEditing);
  popupBrowseDataHeaderMenu->addAction(actionBrowseTableEditDisplayFormat);
}

void Widget::initLayout()
{
  verticalLayout_2 = new QVBoxLayout(this);
  verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
  horizontalLayout = new QHBoxLayout();
  horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
  label = new QLabel(this);
  label->setObjectName(QStringLiteral("label"));
  label->setText("Table");
  horizontalLayout->addWidget(label);

  comboBrowseTable = new QComboBox(this);
  comboBrowseTable->setObjectName(QStringLiteral("comboBrowseTable"));
  QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
  sizePolicy.setHorizontalStretch(1);
  sizePolicy.setVerticalStretch(0);
  sizePolicy.setHeightForWidth(comboBrowseTable->sizePolicy().hasHeightForWidth());
  comboBrowseTable->setSizePolicy(sizePolicy);
  comboBrowseTable->setMinimumSize(QSize(150, 0));
  comboBrowseTable->setMaxVisibleItems(30);
  comboBrowseTable->setSizeAdjustPolicy(QComboBox::AdjustToContents);

  horizontalLayout->addWidget(comboBrowseTable);

  buttonRefresh = new QToolButton(this);
  buttonRefresh->setObjectName(QStringLiteral("buttonRefresh"));
  QIcon icon26;
  icon26.addFile((iconsPath+"view-refresh.png"), QSize(), QIcon::Normal, QIcon::Off);
  buttonRefresh->setIcon(icon26);

  buttonRefresh->setToolTip(QApplication::translate("MainWindow", "Refresh the data in the selected table", 0));
  horizontalLayout->addWidget(buttonRefresh);

  buttonClearFilters = new QToolButton(this);
  buttonClearFilters->setObjectName(QStringLiteral("buttonClearFilters"));
  QIcon icon27;
  icon27.addFile((iconsPath+"clear_filters"), QSize(), QIcon::Normal, QIcon::Off);
  buttonClearFilters->setIcon(icon27);
  buttonClearFilters->setToolTip(QApplication::translate("MainWindow", "Clear all filters", 0));
  horizontalLayout->addWidget(buttonClearFilters);

  horizontalSpacer = new QSpacerItem(200, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

  horizontalLayout->addItem(horizontalSpacer);

  buttonNewRecord = new QPushButton("new record", this);
  buttonNewRecord->setObjectName(QStringLiteral("buttonNewRecord"));
  buttonNewRecord->setToolTip(QApplication::translate("MainWindow", "Insert a new record in the current table", 0));
  horizontalLayout->addWidget(buttonNewRecord);

  buttonDeleteRecord = new QPushButton("delete record", this);
  buttonDeleteRecord->setObjectName(QStringLiteral("buttonDeleteRecord"));
  buttonDeleteRecord->setToolTip(QApplication::translate("MainWindow", "Delete the current record", 0));
  horizontalLayout->addWidget(buttonDeleteRecord); 


  verticalLayout_2->addLayout(horizontalLayout);

  dataTable = new ExtendedTableWidget(this);
  dataTable->setObjectName(QStringLiteral("dataTable"));
  dataTable->setAcceptDrops(true);
  dataTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
  dataTable->setDragDropMode(QAbstractItemView::DragDrop);
  dataTable->setDefaultDropAction(Qt::CopyAction);
  dataTable->setSelectionMode(QAbstractItemView::ContiguousSelection);
  dataTable->horizontalHeader()->setProperty("showSortIndicator", QVariant(true));

  verticalLayout_2->addWidget(dataTable);

  horizontalLayout_2 = new QHBoxLayout();
  horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
  buttonBegin = new QToolButton(this);
  buttonBegin->setObjectName(QStringLiteral("buttonBegin"));
  buttonBegin->setEnabled(false);
  QIcon icon28;
  icon28.addFile((iconsPath+"resultset_first.png"), QSize(), QIcon::Normal, QIcon::Off);
  buttonBegin->setIcon(icon28);
  buttonBegin->setToolTip(QApplication::translate("MainWindow", "<html><head/><body><p>Scroll to the beginning</p></body></html>", 0));
  horizontalLayout_2->addWidget(buttonBegin);

  buttonPrevious = new QToolButton(this);
  buttonPrevious->setObjectName(QStringLiteral("buttonPrevious"));
  buttonPrevious->setEnabled(false);
  QIcon icon29;
  icon29.addFile((iconsPath+"resultset_previous.png"), QSize(), QIcon::Normal, QIcon::Off);
  buttonPrevious->setIcon(icon29);
  buttonPrevious->setToolTip(QApplication::translate("MainWindow", "Scroll one page upwards", 0));
  horizontalLayout_2->addWidget(buttonPrevious);

  labelRecordset = new QLabel(this);
  labelRecordset->setObjectName(QStringLiteral("labelRecordset"));

  horizontalLayout_2->addWidget(labelRecordset);
  buttonNext = new QToolButton(this);
  buttonNext->setObjectName(QStringLiteral("buttonNext"));
  buttonNext->setEnabled(false);


  QIcon icon13;
  icon13.addFile((iconsPath+"resultset_next.png"), QSize(), QIcon::Normal, QIcon::Off);
  buttonNext->setIcon(icon13);
  buttonNext->setToolTip(QApplication::translate("MainWindow", "Scroll one page downwards", 0));

  horizontalLayout_2->addWidget(buttonNext);

  buttonEnd = new QToolButton(this);
  buttonEnd->setObjectName(QStringLiteral("buttonEnd"));
  buttonEnd->setEnabled(false);
  QIcon icon17;
  icon17.addFile((iconsPath+"resultset_last.png"), QSize(), QIcon::Normal, QIcon::Off);
  buttonEnd->setIcon(icon17);
  buttonEnd->setToolTip(QApplication::translate("MainWindow", "Scroll to the end", 0));
  horizontalLayout_2->addWidget(buttonEnd);

  horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

  horizontalLayout_2->addItem(horizontalSpacer_2);

  buttonGoto = new QPushButton("Go to", this);
  buttonGoto->setObjectName(QStringLiteral("buttonGoto"));
  buttonGoto->setToolTip(QApplication::translate("MainWindow", "<html><head/><body><p>Click here to jump to the specified record</p></body></html>", 0));

  horizontalLayout_2->addWidget(buttonGoto);

  editGoto = new QLineEdit(this);
  editGoto->setObjectName(QStringLiteral("editGoto"));
  editGoto->setPlaceholderText("1");
  horizontalLayout_2->addWidget(editGoto);
  editGoto->setValidator(gotoValidator);

  verticalLayout_2->addLayout(horizontalLayout_2);
}

Widget::~Widget()
{
    delete update_timer_;
    
    // delete actionShowRowidColumn;
    // delete actionUnlockViewEditing;
    // delete actionBrowseTableEditDisplayFormat;
    // delete popupBrowseDataHeaderMenu;


    // delete label;
    // delete comboBrowseTable;
    // delete buttonRefresh;
    // delete buttonClearFilters;
    // delete horizontalSpacer;
    // delete horizontalSpacer_2;
    // delete buttonNewRecord;
    // delete buttonDeleteRecord;
    
    // delete buttonBegin;
    // delete buttonPrevious;
    // delete labelRecordset;
    // delete buttonNext;
    // delete buttonEnd;
    // delete buttonGoto;
    // delete editGoto;
    // delete dataTable;
    // delete  gotoValidator;
    // delete  m_browseTableModel;
    // delete dbStructureModel;


    // delete horizontalLayout;
    // delete verticalLayout_2;
    // delete horizontalLayout_2;


    delete gotoValidator;
}

void Widget::requestCollation(const QString& name, int eTextRep)
{
    QMessageBox::StandardButton reply = QMessageBox::question(
                this,
                tr("Collation needed! Proceed?"),
                tr("A table in this database requires a special collation function '%1' "
                   "that this application can't provide without further knowledge.\n"
                   "If you choose to proceed, be aware bad things can happen to your database.\n"
                   "Create a backup!").arg(name), QMessageBox::Yes | QMessageBox::No);
    if(reply == QMessageBox::Yes)
        sqlite3_create_collation(db._db, name.toUtf8(), eTextRep, nullptr, collCompare);
}



sqlb::ObjectIdentifier Widget::currentlyBrowsedTableName() const
{
    return sqlb::ObjectIdentifier(comboBrowseTable->model()->data(dbStructureModel->index(comboBrowseTable->currentIndex(),
                                                                                              DbStructureModel::ColumnSchema,
                                                                                              comboBrowseTable->rootModelIndex())).toString(),
                                  comboBrowseTable->currentData(Qt::EditRole).toString());  // Use the edit role here to make sure we actually get the
                                                                                                // table name without the schema bit in front of it.
}
void Widget::hideColumns(int column, bool hide)
{
    sqlb::ObjectIdentifier tableName = currentlyBrowsedTableName();

    // Select columns to (un)hide
    QSet<int> columns;
    if(column == -1)
    {
         if(dataTable->selectedCols().size() == 0)
             columns.insert(actionBrowseTableEditDisplayFormat->property("clicked_column").toInt());
         else
             columns += dataTable->selectedCols();
    } else {
        columns.insert(column);
    }

    // (Un)hide requested column(s)
    for(int col : columns)
    {
        dataTable->setColumnHidden(col, hide);
        if(!hide)
            dataTable->setColumnWidth(col, dataTable->horizontalHeader()->defaultSectionSize());
        // browseTableSettings[tableName].hiddenColumns[col] = hide;
    }

    // check to see if all the columns are hidden
    bool allHidden = true;
    for(int col = 1; col < dataTable->model()->columnCount(); col++)
    {
        if(!dataTable->isColumnHidden(col))
        {
            allHidden = false;
            break;
        }
    }

    if(allHidden  && dataTable->model()->columnCount() > 1)
        hideColumns(1, false);
}
void Widget::on_actionShowAllColumns_triggered()
{
    for(int col = 1; col < dataTable->model()->columnCount(); col++)
    {
        if(dataTable->isColumnHidden(col))
            hideColumns(col, false);
    }
}
void Widget::enableEditing(bool enable_edit, bool enable_insertdelete)
{
    // Don't enable anything if this is a read only database
    bool edit = enable_edit && !db.readOnly();
    bool insertdelete = enable_insertdelete && !db.readOnly();

    // Apply settings
    buttonNewRecord->setEnabled(insertdelete);
    buttonDeleteRecord->setEnabled(insertdelete);
    dataTable->setEditTriggers(edit ? QAbstractItemView::SelectedClicked | QAbstractItemView::AnyKeyPressed | QAbstractItemView::EditKeyPressed : QAbstractItemView::NoEditTriggers);

}
void Widget::populateStructure()
{
    QString old_table = comboBrowseTable->currentText();

    // Refresh the structure tab
    dbStructureModel->reloadData();
    // Refresh the browse data tab
    comboBrowseTable->setRootModelIndex(dbStructureModel->index(0, 0)); // Show the 'browsable' section of the db structure tree
    int old_table_index = comboBrowseTable->findText(old_table);
    if(old_table_index == -1 && comboBrowseTable->count())      // If the old table couldn't be found anymore but there is another table, select that
        comboBrowseTable->setCurrentIndex(0);
    else if(old_table_index == -1)                                  // If there aren't any tables to be selected anymore, clear the table view
        clearTableBrowser();
    else                                                            // Under normal circumstances just select the old table again
        comboBrowseTable->setCurrentIndex(old_table_index);

    // Cancel here if no database is opened
    if(!db.isOpen())
        return;
}
void Widget::clearTableBrowser()
{
    if (!dataTable->model())
        return;

    dataTable->setModel(nullptr);
    if(qobject_cast<FilterTableHeader*>(dataTable->horizontalHeader()))
        qobject_cast<FilterTableHeader*>(dataTable->horizontalHeader())->generateFilters(0);
}
void Widget::unlockViewEditing(bool unlock, QString pk)
{
    sqlb::ObjectIdentifier currentTable = currentlyBrowsedTableName();

    // If this isn't a view just unlock editing and return
    if(db.getObjectByName(currentTable)->type() != sqlb::Object::View)
    {
        m_browseTableModel->setPseudoPk(QString());
        enableEditing(true, true);
        return;
    }

    // If the view gets unlocked for editing and we don't have a 'primary key' for this view yet, then ask for one
    if(unlock && pk.isEmpty())
    {
        while(true)
        {
            // Ask for a PK
            pk = QInputDialog::getText(this, qApp->applicationName(), tr("Please enter a pseudo-primary key in order to enable editing on this view. "
                                                                         "This should be the name of a unique column in the view."));
            // Cancelled?
            if(pk.isEmpty())
                return;
            // Do some basic testing of the input and if the input appears to be good, go on
            if(db.executeSQL(QString("SELECT %1 FROM %2 LIMIT 1;").arg(sqlb::escapeIdentifier(pk)).arg(currentTable.toString()), false, true))
                break;
        }
    } else if(!unlock) {
        // Locking the view is done by unsetting the pseudo-primary key
        pk.clear();
    }
    // (De)activate editing
    enableEditing(unlock, false);
    m_browseTableModel->setPseudoPk(pk);

    // Update checked status of the popup menu action
    actionUnlockViewEditing->setChecked(unlock);

    // Save settings for this table
    // browseTableSettings[currentTable].unlockViewPk = pk;
}

void Widget::showRowidColumn(bool show)
{
    // Block all signals from the horizontal header. Otherwise the QHeaderView::sectionResized signal causes us trouble
    dataTable->horizontalHeader()->blockSignals(true);

    // WORKAROUND
    // Set the opposite hidden/visible status of what we actually want for the rowid column. This is to work around a Qt bug which
    // is present in at least version 5.7.1. The problem is this: when you browse a table/view with n colums, then switch to a table/view
    // with less than n columns, you'll be able to resize the first (hidden!) column by resizing the section to the left of the first visible
    // column. By doing so the table view gets messed up. But even when not resizing the first hidden column, tab-ing through the fields
    // will stop at the not-so-much-hidden rowid column, too. All this can be fixed by this line. I haven't managed to find another workaround
    // or way to fix this yet.
    dataTable->setColumnHidden(0, show);

    // Show/hide rowid column
    dataTable->setColumnHidden(0, !show);

    // Update checked status of the popup menu action
    actionShowRowidColumn->setChecked(show);

    // Save settings for this table
    sqlb::ObjectIdentifier current_table = currentlyBrowsedTableName();
    // browseTableSettings[current_table].showRowid = show;

    // Update the filter row
    qobject_cast<FilterTableHeader*>(dataTable->horizontalHeader())->generateFilters(m_browseTableModel->columnCount(), show);

    // Re-enable signals
    dataTable->horizontalHeader()->blockSignals(false);

    dataTable->update();
}
void Widget::populateTable()
{

    // Remove the model-view link if the table name is empty in order to remove any data from the view
    if(comboBrowseTable->model()->rowCount(comboBrowseTable->rootModelIndex()) == 0)
    {
        clearTableBrowser();
        return;
    }

    QApplication::setOverrideCursor(Qt::WaitCursor);

    // Get current table name
    sqlb::ObjectIdentifier tablename = currentlyBrowsedTableName();

    // Set model
    bool reconnectSelectionSignals = false;
    if(dataTable->model() == nullptr)
        reconnectSelectionSignals = true;
    dataTable->setModel(m_browseTableModel);
    if(reconnectSelectionSignals)
    {
        connect(dataTable->selectionModel(), SIGNAL(currentChanged(QModelIndex,QModelIndex)), this, SLOT(dataTableSelectionChanged(QModelIndex)));

        // Lambda function for updating the delete record button to reflect number of selected records
        connect(dataTable->selectionModel(), &QItemSelectionModel::selectionChanged, [this](const QItemSelection&, const QItemSelection&) {
            // NOTE: We're assuming here that the selection is always contiguous, i.e. that there are never two selected rows with a non-selected
            // row in between.
            int rows = 0;
            if(dataTable->selectionModel()->selectedIndexes().count())
                rows = dataTable->selectionModel()->selectedIndexes().last().row() - dataTable->selectionModel()->selectedIndexes().first().row() + 1;

            if(rows > 1)
                buttonDeleteRecord->setText(tr("Delete records"));
            else
                buttonDeleteRecord->setText(tr("Delete record"));
        });
    }

    // Set table name and apply default display format settings
    m_browseTableModel->setTable(tablename, 0, Qt::AscendingOrder);

    // There aren't any information stored for this table yet, so use some default values

    // Hide rowid column. Needs to be done before the column widths setting because of the workaround in there
    showRowidColumn(false);

    // Unhide all columns by default
    on_actionShowAllColumns_triggered();

    // Enable editing in general, but lock view editing
    unlockViewEditing(false);

    // Column widths
    for(int i=1;i<m_browseTableModel->columnCount();i++)
        dataTable->setColumnWidth(i, dataTable->horizontalHeader()->defaultSectionSize());

    // Sorting
    dataTable->filterHeader()->setSortIndicator(0, Qt::AscendingOrder);
    dataTable->horizontalHeader()->setStretchLastSection(true);
    dataTable->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    dataTable->horizontalHeader()->setSectionResizeMode(6, QHeaderView::ResizeToContents);
    dataTable->horizontalHeader()->setSectionResizeMode(7, QHeaderView::ResizeToContents);

    if(db.getObjectByName(currentlyBrowsedTableName())->type() == sqlb::Object::Table)
    {
        // Table
        actionUnlockViewEditing->setVisible(false);
        actionShowRowidColumn->setVisible(true);
    } else {
        // View
        actionUnlockViewEditing->setVisible(true);
        actionShowRowidColumn->setVisible(false);
    }

    QApplication::restoreOverrideCursor();
}

bool Widget::fileClose()
{
    // Close the database but stop the closing process here if the user pressed the cancel button in there
    if(!db.close())
        return false;

    // setWindowTitle(QApplication::applicationName());
    // loadPragmas();
    // statusEncryptionLabel->setVisible(false);
    // statusReadOnlyLabel->setVisible(false);

    // // Reset the model for the Browse tab
    // m_browseTableModel->reset();

    // // Remove all stored table information browse data tab
    // browseTableSettings.clear();
    // defaultBrowseTableEncoding = QString();

    // // Clear edit dock
    // editDock->setCurrentIndex(QModelIndex());

    // // Reset the recordset label inside the Browse tab now
    // setRecordsetLabel();

    // // Reset the plot dock model
    // plotDock->updatePlot(nullptr);

    // activateFields(false);

    // // Clear the SQL Log
    // editLogApplication->clear();
    // editLogUser->clear();

    return true;
}
bool Widget::fileOpen(const QString& fileName, bool dontAddToRecentFiles, bool readOnly)
{
    bool retval = false;

    QString wFile = fileName;
    if (!QFile::exists(wFile))
    {
        wFile = FileDialog::getOpenFileName(
                    this,
                    tr("Choose a database file")
#ifndef Q_OS_MAC // Filters on OS X are buggy
                    , FileDialog::getSqlDatabaseFileFilter()
#endif
                    );
    }
    if(QFile::exists(wFile) )
    {
        // Close the database. If the user didn't want to close it, though, stop here
        if (db.isOpen())
            if(!fileClose())
                return false;

        // Try opening it as a project file first
        // if(loadProject(wFile, readOnly))
        // {
        //     retval = true;
        // } else 
        {
            // No project file; so it should be a database file
            if(db.open(wFile, readOnly))
            {
                // Close all open but empty SQL tabs
                // for(int i=tabSqlAreas->count()-1;i>=0;i--)
                // {
                //     if(qobject_cast<SqlExecutionArea*>(tabSqlAreas->widget(i))->getSql().trimmed().isEmpty())
                //         closeSqlTab(i, true);
                // }

                // statusEncodingLabel->setText(db.getPragma("encoding"));
                // statusEncryptionLabel->setVisible(db.encrypted());
                // statusReadOnlyLabel->setVisible(db.readOnly());
                setCurrentFile(wFile);
                // if(!dontAddToRecentFiles)
                //     addToRecentFilesMenu(wFile);
                // openSqlTab(true);
                // loadExtensionsFromSettings();
                // if(mainTab->currentIndex() == BrowseTab)
                    populateTable();
                // else if(mainTab->currentIndex() == PragmaTab)
                    // loadPragmas();
                retval = true;
            } else {
                QMessageBox::warning(this, qApp->applicationName(), tr("Could not open database file.\nReason: %1").arg(db.lastError()));
                return false;
            }
        }
    }

    return retval;
}

void Widget::activateFields(bool enable)
{
    bool write = !db.readOnly();
    buttonNext->setEnabled(enable);
    buttonPrevious->setEnabled(enable);
    buttonBegin->setEnabled(enable);
    buttonEnd->setEnabled(enable);

    buttonGoto->setEnabled(enable);
    editGoto->setEnabled(enable);
    buttonRefresh->setEnabled(enable);
    buttonDeleteRecord->setEnabled(enable && write);
    buttonNewRecord->setEnabled(enable && write);

    buttonClearFilters->setEnabled(enable);
}
void Widget::dbState( bool dirty )
{

}

void Widget::setCurrentFile(const QString &fileName)
{
    setWindowFilePath(fileName);
    setWindowTitle(QApplication::applicationName() + " - " + QDir::toNativeSeparators(fileName));
    activateFields(true);
    dbState(db.getDirty());
}
void Widget::updateFilter(int column, const QString& value)
{
    m_browseTableModel->updateFilter(column, value);
//    browseTableSettings[currentlyBrowsedTableName()].filterValues[column] = value;
    setRecordsetLabel();
}
void Widget::resizeEvent(QResizeEvent*)
{
    setRecordsetLabel();
}
void Widget::setRecordsetLabel()
{
    // Get all the numbers, i.e. the number of the first row and the last row as well as the total number of rows
    int from = dataTable->verticalHeader()->visualIndexAt(0) + 1;
    int total = m_browseTableModel->totalRowCount();
    int to = dataTable->verticalHeader()->visualIndexAt(dataTable->height()) - 1;
    if (to == -2)
        to = total;

    // Update the validator of the goto row field
    gotoValidator->setRange(0, total);

    // Update the label showing the current position
    labelRecordset->setText(tr("%1 - %2 of %3").arg(from).arg(to).arg(total));
}

void Widget::navigateGoto()
{
    int row = editGoto->text().toInt();
    if(row <= 0)
        row = 1;
    if(row > m_browseTableModel->totalRowCount())
        row = m_browseTableModel->totalRowCount();

    selectTableLine(row - 1);
    editGoto->setText(QString::number(row));
}
void Widget::selectTableLine(int lineToSelect)
{
    // Are there even that many lines?
    if(lineToSelect >= m_browseTableModel->totalRowCount())
        return;

    QApplication::setOverrideCursor( Qt::WaitCursor );
    // Make sure this line has already been fetched
    while(lineToSelect >= m_browseTableModel->rowCount() && m_browseTableModel->canFetchMore())
          m_browseTableModel->fetchMore();

    // Select it
    dataTable->clearSelection();
    dataTable->selectRow(lineToSelect);
    dataTable->scrollTo(dataTable->currentIndex(), QAbstractItemView::PositionAtTop);
    QApplication::restoreOverrideCursor();
}
void Widget::selectTableLines(int firstLine, int count)
{
    int lastLine = firstLine+count-1;
    // Are there even that many lines?
    if(lastLine >= m_browseTableModel->totalRowCount())
        return;

    selectTableLine(firstLine);

    QModelIndex topLeft = dataTable->model()->index(firstLine, 0);
    QModelIndex bottomRight = dataTable->model()->index(lastLine, dataTable->model()->columnCount()-1);

    dataTable->selectionModel()->select(QItemSelection(topLeft, bottomRight), QItemSelectionModel::Select | QItemSelectionModel::Rows);
}
void Widget::refresh()
{
    // Refresh the schema and reload the current table
    db.updateSchema();
    populateTable();
}

void Widget::navigatePrevious()
{
    int curRow = dataTable->currentIndex().row();
    curRow -= dataTable->numVisibleRows() - 1;
    if(curRow < 0)
        curRow = 0;
    selectTableLine(curRow);
}


void Widget::navigateNext()
{
    int curRow = dataTable->currentIndex().row();
    curRow += dataTable->numVisibleRows() - 1;
    if(curRow >= m_browseTableModel->totalRowCount())
        curRow = m_browseTableModel->totalRowCount() - 1;
    selectTableLine(curRow);
}

void Widget::navigateBegin()
{
    selectTableLine(0);
}

void Widget::navigateEnd()
{
    selectTableLine(m_browseTableModel->totalRowCount()-1);
}

void Widget::on_buttonClearFilters_clicked()
{
    dataTable->filterHeader()->clearFilters();
}
void Widget::addRecord()
{
    int row = m_browseTableModel->rowCount();
    if(m_browseTableModel->insertRow(row))
    {
        selectTableLine(row);
    } else {
        QMessageBox::warning(this, QApplication::applicationName(), tr("Error adding record:\n") + db.lastError());
    }
}
void Widget::deleteRecord()
{
    if(dataTable->selectionModel()->hasSelection())
    {
        // If only filter header is selected
        if(dataTable->selectionModel()->selectedIndexes().isEmpty())
            return;

        int old_row = dataTable->currentIndex().row();
        while(dataTable->selectionModel()->hasSelection())
        {
            int first_selected_row = dataTable->selectionModel()->selectedIndexes().first().row();
            int last_selected_row = dataTable->selectionModel()->selectedIndexes().last().row();
            int selected_rows_count = last_selected_row - first_selected_row + 1;
            if(!m_browseTableModel->removeRows(first_selected_row, selected_rows_count))
            {
                QMessageBox::warning(this, QApplication::applicationName(), tr("Error deleting record:\n%1").arg(db.lastError()));
                break;
            }
        }

        if(old_row > m_browseTableModel->totalRowCount())
            old_row = m_browseTableModel->totalRowCount();
        selectTableLine(old_row);
    } else {
        QMessageBox::information( this, QApplication::applicationName(), tr("Please select a record first"));
    }
}


void Widget::editDataColumnDisplayFormat()
{

}

void Widget::browseTableHeaderClicked(int logicalindex)
{
    // Abort if there is more than one column selected because this tells us that the user pretty sure wants to do a range selection instead of sorting data
    if(dataTable->selectionModel()->selectedColumns().count() > 1)
        return;
    dataTable->setCurrentIndex(dataTable->currentIndex().sibling(0, logicalindex));
}
void Widget::dataTableSelectionChanged(const QModelIndex& index)
{

}
}