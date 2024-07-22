#ifndef PARAMETERTREEITEM_H
#define PARAMETERTREEITEM_H

#include <QVariant>
#include <QList>

class ParameterTreeItem
{

public:

    typedef ParameterTreeItem TreeItem;

    explicit ParameterTreeItem(QString name, QString type,
                               ParameterTreeItem *parentItem = nullptr);

    void appendChild(std::unique_ptr<ParameterTreeItem> &&child);

    ParameterTreeItem *child(int row);
    int childCount() const;
    int columnCount() const;
    QVariant data(int column) const;
    int row() const;
    ParameterTreeItem *parentItem();
    ParameterTreeItem *findChild(QString name);

private:
    std::vector<std::unique_ptr<ParameterTreeItem>> m_childItems;
    QVariantList m_itemData;
    ParameterTreeItem *m_parentItem;
};

#endif // PARAMETERTREEITEM_H
