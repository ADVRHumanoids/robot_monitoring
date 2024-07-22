#include "parameter_tree_item.h"


ParameterTreeItem::ParameterTreeItem(QString name,
                                     QString type,
                                     ParameterTreeItem *parentItem):
    m_parentItem(parentItem)
{
    m_itemData.append(name);
    m_itemData.append(type);

    if(!parentItem) return;
    qInfo() << name << type << parentItem->m_itemData;
}

void ParameterTreeItem::appendChild(std::unique_ptr<ParameterTreeItem> &&child)
{
    m_childItems.push_back(std::move(child));
}

ParameterTreeItem *ParameterTreeItem::child(int row)
{
    return row >= 0 && row < childCount() ? m_childItems.at(row).get() : nullptr;
}

int ParameterTreeItem::childCount() const
{
    return int(m_childItems.size());
}

int ParameterTreeItem::columnCount() const
{
    return int(m_itemData.count());
}

QVariant ParameterTreeItem::data(int column) const
{
    return m_itemData.value(column);
}

int ParameterTreeItem::row() const
{
    if (m_parentItem == nullptr)
        return 0;
    const auto it = std::find_if(m_parentItem->m_childItems.cbegin(), m_parentItem->m_childItems.cend(),
                                 [this](const std::unique_ptr<TreeItem> &treeItem) {
                                     return treeItem.get() == this;
                                 });

    if (it != m_parentItem->m_childItems.cend())
        return std::distance(m_parentItem->m_childItems.cbegin(), it);
    Q_ASSERT(false); // should not happen
    return -1;
}

ParameterTreeItem *ParameterTreeItem::parentItem()
{
    return m_parentItem;
}

ParameterTreeItem *ParameterTreeItem::findChild(QString name)
{
    for(auto& c : m_childItems)
    {
        if(c->data(0) == name)
        {
            return c.get();
        }
    }

    return nullptr;
}
