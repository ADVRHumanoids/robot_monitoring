// Copyright (C) 2024 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "diagnosticfiltermodel.h"
#include "treemodel.h"
#include <QVector>

using namespace Qt::StringLiterals;

DiagnosticFilterModel::DiagnosticFilterModel(QObject *parent)
    : QSortFilterProxyModel(parent)
{
    setDynamicSortFilter(true);
}

QString DiagnosticFilterModel::filterText() const
{
    return m_filterText;
}

void DiagnosticFilterModel::setFilterText(const QString &filterText)
{
    const auto trimmedFilterText = filterText.trimmed();
    if (m_filterText == trimmedFilterText)
        return;

    beginFilterChange();
    m_filterText = trimmedFilterText;
    endFilterChange(Direction::Rows);
    emit filterTextChanged();
}

int DiagnosticFilterModel::minimumLevel() const
{
    return m_minimumLevel;
}

void DiagnosticFilterModel::setMinimumLevel(int minimumLevel)
{
    if (m_minimumLevel == minimumLevel)
        return;

    beginFilterChange();
    m_minimumLevel = minimumLevel;
    endFilterChange(Direction::Rows);
    emit minimumLevelChanged();
}

QModelIndex DiagnosticFilterModel::indexForPath(const QString &path, int column) const
{
    const auto *treeModel = qobject_cast<const TreeModel *>(sourceModel());
    if (!treeModel)
        return {};

    const auto sourceIndex = treeModel->indexForPath(path, column);
    return sourceIndex.isValid() ? mapFromSource(sourceIndex) : QModelIndex{};
}

QStringList DiagnosticFilterModel::expandablePaths() const
{
    QStringList paths;
    QVector<QModelIndex> indexes;

    for (int row = 0; row < rowCount(); ++row)
        indexes.append(index(row, 0));

    while (!indexes.isEmpty()) {
        const auto currentIndex = indexes.takeLast();
        const int childCount = rowCount(currentIndex);
        if (childCount <= 0)
            continue;

        const auto path = data(currentIndex, TreeModel::PathRole).toString();
        if (!path.isEmpty())
            paths.append(path);

        for (int row = 0; row < childCount; ++row)
            indexes.append(index(row, 0, currentIndex));
    }

    return paths;
}

bool DiagnosticFilterModel::filterAcceptsRow(int sourceRow, const QModelIndex &sourceParent) const
{
    if (m_filterText.isEmpty() && m_minimumLevel < 0)
        return true;
    if (!sourceModel())
        return false;

    const auto sourceIndex = sourceModel()->index(sourceRow, 0, sourceParent);
    return rowMatches(sourceIndex) || hasAcceptedDescendant(sourceIndex);
}

bool DiagnosticFilterModel::rowMatches(const QModelIndex &sourceIndex) const
{
    return textMatches(sourceIndex) && levelMatches(sourceIndex);
}

bool DiagnosticFilterModel::textMatches(const QModelIndex &sourceIndex) const
{
    const auto matches = [this, &sourceIndex](int role) {
        return sourceModel()->data(sourceIndex, role).toString().contains(m_filterText, Qt::CaseInsensitive);
    };

    return m_filterText.isEmpty()
           || matches(TreeModel::NameRole)
           || matches(TreeModel::PathRole)
           || matches(TreeModel::LevelRole)
           || matches(TreeModel::MessageRole)
           || matches(TreeModel::HardwareIdRole)
           || matches(TreeModel::MetricSummaryRole);
}

bool DiagnosticFilterModel::levelMatches(const QModelIndex &sourceIndex) const
{
    if (m_minimumLevel < 0)
        return true;

    const auto level = sourceModel()->data(sourceIndex, TreeModel::LevelRole);
    return level.isValid() && level.toInt() >= m_minimumLevel;
}

bool DiagnosticFilterModel::hasAcceptedDescendant(const QModelIndex &sourceIndex) const
{
    const int childCount = sourceModel()->rowCount(sourceIndex);
    for (int row = 0; row < childCount; ++row) {
        const auto childIndex = sourceModel()->index(row, 0, sourceIndex);
        if (rowMatches(childIndex) || hasAcceptedDescendant(childIndex))
            return true;
    }

    return false;
}
