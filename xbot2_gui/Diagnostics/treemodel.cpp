// Copyright (C) 2024 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

/*
    treemodel.cpp

    Provides a simple tree model to show how to create and use hierarchical
    models.
*/

#include "treemodel.h"
#include "treeitem.h"
#include <QStringList>

using namespace Qt::StringLiterals;

//! [0]
TreeModel::TreeModel(QObject *parent)
    : QAbstractItemModel(parent)
    , rootItem(createRootItem())
{}
//! [0]

//! [1]
TreeModel::~TreeModel() = default;
//! [1]

//! [2]
int TreeModel::columnCount(const QModelIndex &parent) const
{
    if (parent.isValid()) {
        const auto *item = static_cast<const TreeItem*>(parent.internalPointer());
        return item->columnCount();
    }
    return rootItem->columnCount();
}
//! [2]

//! [3]
QVariant TreeModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return {};

    const auto *item = static_cast<const TreeItem*>(index.internalPointer());
    if (role == Qt::DisplayRole)
        return item->data(index.column());

    return item->namedData(role);
}
//! [3]

//! [4]
Qt::ItemFlags TreeModel::flags(const QModelIndex &index) const
{
    return index.isValid()
    ? QAbstractItemModel::flags(index) : Qt::ItemFlags(Qt::NoItemFlags);
}
//! [4]

//! [5]
QVariant TreeModel::headerData(int section, Qt::Orientation orientation,
                               int role) const
{
    if (orientation != Qt::Horizontal || role != Qt::DisplayRole)
        return {};

    switch (section) {
    case TreeItem::NameColumn:
        return tr("Name");
    case TreeItem::LevelColumn:
        return tr("Level");
    case TreeItem::MessageColumn:
        return tr("Message");
    case TreeItem::HardwareIdColumn:
        return tr("Hardware ID");
    case TreeItem::MetricsColumn:
        return tr("Metrics");
    default:
        return {};
    }
}
//! [5]

//! [6]
QModelIndex TreeModel::index(int row, int column, const QModelIndex &parent) const
{
    if (!hasIndex(row, column, parent))
        return {};

    TreeItem *parentItem = parent.isValid()
                               ? static_cast<TreeItem*>(parent.internalPointer())
                               : rootItem.get();

    if (auto *childItem = parentItem->child(row))
        return createIndex(row, column, childItem);
    return {};
}
//! [6]

//! [7]
QModelIndex TreeModel::parent(const QModelIndex &index) const
{
    if (!index.isValid())
        return {};

    auto *childItem = static_cast<TreeItem*>(index.internalPointer());
    TreeItem *parentItem = childItem->parentItem();

    return parentItem != rootItem.get()
               ? createIndex(parentItem->row(), 0, parentItem) : QModelIndex{};
}
//! [7]

//! [8]
int TreeModel::rowCount(const QModelIndex &parent) const
{
    if (parent.column() > 0)
        return 0;

    const TreeItem *parentItem = parent.isValid()
                                     ? static_cast<const TreeItem*>(parent.internalPointer())
                                     : rootItem.get();

    return parentItem->childCount();
}
//! [8]

QHash<int, QByteArray> TreeModel::roleNames() const
{
    auto roles = QAbstractItemModel::roleNames();
    roles[NameRole] = "name";
    roles[PathRole] = "path";
    roles[LevelRole] = "level";
    roles[MessageRole] = "message";
    roles[HardwareIdRole] = "hardwareId";
    roles[MetricsRole] = "metrics";
    roles[MetricSummaryRole] = "metricSummary";
    roles[IsLeafRole] = "isLeaf";
    return roles;
}

void TreeModel::loadFromDiagnostics(const QVariantMap &diagnostics)
{
    beginResetModel();
    rootItem = createRootItem();
    setupModelData(diagnostics, rootItem.get());
    endResetModel();
}

std::unique_ptr<TreeItem> TreeModel::createRootItem()
{
    return std::make_unique<TreeItem>("Diagnostics"_L1);
}

void TreeModel::setupModelData(const QVariantMap &diagnostics, TreeItem *parent)
{
    const auto statuses = diagnostics.value("status"_L1).toList();

    for (const auto &statusValue : statuses) {
        const auto status = statusValue.toMap();
        const auto fullName = status.value("name"_L1).toString();
        const auto pathSegments = fullName.split(u'/', Qt::SkipEmptyParts);
        if (pathSegments.isEmpty())
            continue;

        TreeItem *currentItem = parent;
        QString currentPath;

        for (const auto &segment : pathSegments) {
            currentPath += "/"_L1 + segment;

            if (auto *child = currentItem->childByName(segment)) {
                currentItem = child;
            } else {
                currentItem = currentItem->appendChild(
                    std::make_unique<TreeItem>(segment, currentPath, currentItem));
            }
        }

        QVariantList metrics;
        const auto values = status.value("values"_L1).toList();
        metrics.reserve(values.size());

        for (const auto &metricValue : values) {
            const auto metric = metricValue.toMap();
            QVariantMap metricMap;
            metricMap.insert("key"_L1, metric.value("key"_L1).toString());
            metricMap.insert("value"_L1, metric.value("value"_L1).toString());
            metrics.append(metricMap);
        }

        const int level = status.contains("level"_L1) ? status.value("level"_L1).toInt() : -1;
        currentItem->setDiagnostics(level,
                                    status.value("message"_L1).toString(),
                                    status.value("hardware_id"_L1).toString(),
                                    metrics);
    }

    parent->sortChildrenRecursively();
}
