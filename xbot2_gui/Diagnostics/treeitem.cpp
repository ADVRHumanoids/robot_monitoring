// Copyright (C) 2024 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

/*
    treeitem.cpp

    A container for items of data supplied by the simple tree model.
*/

#include "treeitem.h"
#include "treemodel.h"
#include <QStringList>
#include <algorithm>

using namespace Qt::StringLiterals;

//! [0]
TreeItem::TreeItem(QString name, QString path, TreeItem *parent)
    : m_name(std::move(name))
    , m_path(std::move(path))
    , m_parentItem(parent)
{}
//! [0]

//! [1]
TreeItem *TreeItem::appendChild(std::unique_ptr<TreeItem> &&child)
{
    auto *childItem = child.get();
    m_childItems.push_back(std::move(child));
    return childItem;
}
//! [1]

//! [2]
TreeItem *TreeItem::child(int row)
{
    return row >= 0 && row < childCount() ? m_childItems.at(row).get() : nullptr;
}
//! [2]

const TreeItem *TreeItem::child(int row) const
{
    return row >= 0 && row < childCount() ? m_childItems.at(row).get() : nullptr;
}

TreeItem *TreeItem::childByName(const QString &name)
{
    const auto it = std::find_if(m_childItems.begin(), m_childItems.end(),
                                 [&name](const std::unique_ptr<TreeItem> &treeItem) {
                                     return treeItem->m_name == name;
                                 });
    return it != m_childItems.end() ? it->get() : nullptr;
}

const TreeItem *TreeItem::childByName(const QString &name) const
{
    const auto it = std::find_if(m_childItems.cbegin(), m_childItems.cend(),
                                 [&name](const std::unique_ptr<TreeItem> &treeItem) {
                                     return treeItem->m_name == name;
                                 });
    return it != m_childItems.cend() ? it->get() : nullptr;
}

//! [3]
int TreeItem::childCount() const
{
    return int(m_childItems.size());
}
//! [3]

//! [4]
int TreeItem::columnCount() const
{
    return ColumnCount;
}
//! [4]

//! [5]
QVariant TreeItem::data(int column) const
{
    switch (column) {
    case NameColumn:
        return m_name;
    case LevelColumn:
        return m_level >= 0 ? QVariant(m_level) : QVariant{};
    case MessageColumn:
        return m_message;
    case HardwareIdColumn:
        return m_hardwareId;
    case MetricsColumn:
        return metricsSummary();
    default:
        return {};
    }
}
//! [5]

QVariant TreeItem::namedData(int role) const
{
    switch (role) {
    case TreeModel::NameRole:
        return m_name;
    case TreeModel::PathRole:
        return m_path;
    case TreeModel::LevelRole:
        return m_level >= 0 ? QVariant(m_level) : QVariant{};
    case TreeModel::MessageRole:
        return m_message;
    case TreeModel::HardwareIdRole:
        return m_hardwareId;
    case TreeModel::MetricsRole:
        return m_metrics;
    case TreeModel::MetricSummaryRole:
        return metricsSummary();
    case TreeModel::IsLeafRole:
        return childCount() == 0;
    default:
        return {};
    }
}

//! [6]
TreeItem *TreeItem::parentItem()
{
    return m_parentItem;
}
//! [6]

//! [7]
int TreeItem::row() const
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
//! [7]

void TreeItem::setDiagnostics(int level, QString message, QString hardwareId, QVariantList metrics)
{
    m_level = level;
    m_message = std::move(message);
    m_hardwareId = std::move(hardwareId);
    m_metrics = std::move(metrics);
}

void TreeItem::sortChildrenRecursively()
{
    std::sort(m_childItems.begin(), m_childItems.end(),
              [](const std::unique_ptr<TreeItem> &left,
                 const std::unique_ptr<TreeItem> &right) {
                  return QString::localeAwareCompare(left->m_name, right->m_name) < 0;
              });

    for (const auto &child : m_childItems)
        child->sortChildrenRecursively();
}

void TreeItem::collectActiveIssues(QVariantList *issues) const
{
    if (m_childItems.empty() && m_level >= 1) {
        QVariantMap issue;
        issue.insert("level"_L1, m_level);
        issue.insert("path"_L1, m_path);
        issue.insert("message"_L1, m_message);
        issues->append(issue);
        return;
    }

    for (const auto &child : m_childItems)
        child->collectActiveIssues(issues);
}

QString TreeItem::metricsSummary() const
{
    QStringList pairs;
    pairs.reserve(m_metrics.size());

    for (const auto &metric : m_metrics) {
        const auto metricMap = metric.toMap();
        const auto key = metricMap.value("key"_L1).toString();
        const auto value = metricMap.value("value"_L1).toString();
        if (!key.isEmpty())
            pairs.append(key + "="_L1 + value);
    }

    return pairs.join(", "_L1);
}
