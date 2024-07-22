#include "parameter_tree_model.h"

ParameterTreeModel::ParameterTreeModel()
{
    QStringList paramNames = {"/xbot/boh",
                  "/xbot/hal/nonso",
                  "/robot_description",
                  "/robot_description_semantic",
                  "/homing/time"};

    QStringList paramTypes = {"string", "string", "int", "Eigen::VectorXd", "double"};

    generateModel(paramNames, paramTypes);
}

void ParameterTreeModel::generateModel(QStringList paramNames, QStringList paramTypes)
{
    beginResetModel();

    rootItem = std::make_unique<TreeItem>("/", "--");

    for(int i = 0; i < paramNames.size(); i++)
    {
        auto pName = paramNames[i];
        auto pType = paramTypes[i];

        auto pathItems = pName.split("/");

        auto * currentItem = rootItem.get();

        for(int j = 0; j < pathItems.size() - 1; j++)
        {
            auto nsItem = currentItem->findChild(pathItems[j]);

            if(!nsItem)
            {
                currentItem->appendChild(std::make_unique<TreeItem>(pathItems[j], "--", currentItem));
                nsItem = currentItem->findChild(pathItems[j]);
            }

            currentItem = nsItem;
        }

        currentItem->appendChild(std::make_unique<TreeItem>(pName, pType, currentItem));

    }

    endResetModel();
}

QModelIndex ParameterTreeModel::index(int row, int column, const QModelIndex &parent) const
{
    if (!hasIndex(row, column, parent))
    {
        return {};
    }

    TreeItem *parentItem = parent.isValid()
                               ? static_cast<TreeItem*>(parent.internalPointer())
                               : rootItem.get();

    if (auto *childItem = parentItem->child(row))
    {
        qInfo() << "createIndex" << row << column << childItem->data(0);
        return createIndex(row, column, childItem);
    }

    return {};
}

QModelIndex ParameterTreeModel::parent(const QModelIndex &index) const
{
    if (!index.isValid())
    {
        return {};
    }

    auto *childItem = static_cast<TreeItem*>(index.internalPointer());

    TreeItem *parentItem = childItem->parentItem();

    return parentItem != rootItem.get()
               ? createIndex(parentItem->row(), 0, parentItem) : QModelIndex{};
}

int ParameterTreeModel::rowCount(const QModelIndex &parent) const
{

    if (parent.column() > 0)
        return 0;

    const TreeItem *parentItem = parent.isValid()
                                     ? static_cast<const TreeItem*>(parent.internalPointer())
                                     : rootItem.get();

    return parentItem->childCount();
}

int ParameterTreeModel::columnCount(const QModelIndex &parent) const
{
    if (parent.isValid())
        return static_cast<TreeItem*>(parent.internalPointer())->columnCount();
    return rootItem->columnCount();
}

QVariant ParameterTreeModel::data(const QModelIndex &index, int role) const
{
    qInfo() << __func__ << index.row() << index.column() << role;
    if (!index.isValid() || role != Qt::DisplayRole)
        return {};

    const auto *item = static_cast<const TreeItem*>(index.internalPointer());
    return item->data(index.column());
}


Qt::ItemFlags ParameterTreeModel::flags(const QModelIndex &index) const
{
    return index.isValid()
               ? QAbstractItemModel::flags(index) : Qt::ItemFlags(Qt::NoItemFlags);
}


// QHash<int, QByteArray> ParameterTreeModel::roleNames() const
// {
//     QHash<int, QByteArray> ret;
//     ret[ParamName] = "paramName";
//     ret[ParamType] = "paramType";
//     return ret;
// }
