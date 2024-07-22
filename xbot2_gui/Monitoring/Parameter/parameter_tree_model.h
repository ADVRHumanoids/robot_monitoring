#ifndef PARAMETERTREEMODEL_H
#define PARAMETERTREEMODEL_H

#include <QAbstractItemModel>
#include "parameter_tree_item.h"
#include <QQmlComponent>

class ParameterTreeModel : public QAbstractItemModel
{
    Q_OBJECT

public:

    QML_ELEMENT

    typedef ParameterTreeItem TreeItem;

    Q_DISABLE_COPY_MOVE(ParameterTreeModel)

    enum Roles {
        ParamName = Qt::UserRole + 1,
        ParamType
    };

    ParameterTreeModel();
    Q_INVOKABLE void generateModel(QStringList paramNames,
                                   QStringList paramTypes);
    QModelIndex index(int row, int column, const QModelIndex &parent) const override;
    QModelIndex parent(const QModelIndex &child) const override;
    int rowCount(const QModelIndex &parent) const override;
    int columnCount(const QModelIndex &parent) const override;
    QVariant data(const QModelIndex &index, int role) const override;
    Qt::ItemFlags flags(const QModelIndex &index) const override;
    // QHash<int, QByteArray> roleNames() const override;

private:

    std::unique_ptr<TreeItem> rootItem;

};

#endif // PARAMETERTREEMODEL_H
