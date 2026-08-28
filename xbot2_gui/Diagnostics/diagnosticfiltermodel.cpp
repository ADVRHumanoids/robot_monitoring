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

QVariantList DiagnosticFilterModel::allowedLevels() const
{
    return m_allowedLevels;
}

void DiagnosticFilterModel::setAllowedLevels(const QVariantList &allowedLevels)
{
    QVariantList normalizedLevels;
    for (const auto &value : allowedLevels) {
        bool isLevel = false;
        const int level = value.toInt(&isLevel);
        if (isLevel && !normalizedLevels.contains(level))
            normalizedLevels.append(level);
    }

    if (m_allowedLevels == normalizedLevels)
        return;

    beginFilterChange();
    m_allowedLevels = normalizedLevels;
    const bool minimumLevelWasChanged = m_minimumLevel != -1;
    m_minimumLevel = -1;
    endFilterChange(Direction::Rows);
    invalidate();
    emit allowedLevelsChanged();
    if (minimumLevelWasChanged)
        emit minimumLevelChanged();
}

int DiagnosticFilterModel::minimumLevel() const
{
    return m_minimumLevel;
}

void DiagnosticFilterModel::setMinimumLevel(int minimumLevel)
{
    QVariantList allowedLevels;
    for (int level = qMax(0, minimumLevel); level <= 3; ++level)
        allowedLevels.append(level);

    if (m_minimumLevel == minimumLevel && m_allowedLevels == allowedLevels)
        return;

    beginFilterChange();
    m_minimumLevel = minimumLevel;
    const bool allowedLevelsWereChanged = m_allowedLevels != allowedLevels;
    m_allowedLevels = allowedLevels;
    endFilterChange(Direction::Rows);
    invalidate();
    emit minimumLevelChanged();
    if (allowedLevelsWereChanged)
        emit allowedLevelsChanged();
}

bool DiagnosticFilterModel::isLevelEnabled(int level) const
{
    return m_allowedLevels.contains(level);
}

void DiagnosticFilterModel::enableLevel(int level, bool enabled)
{
    const bool isEnabled = isLevelEnabled(level);
    if (isEnabled == enabled)
        return;

    auto allowedLevels = m_allowedLevels;
    if (enabled)
        allowedLevels.append(level);
    else
        allowedLevels.removeAll(level);

    setAllowedLevels(allowedLevels);
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
    if (m_filterText.isEmpty() && m_allowedLevels.isEmpty())
        return false;
    if (m_filterText.isEmpty() && m_allowedLevels == QVariantList {0, 1, 2, 3})
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
    const auto level = sourceModel()->data(sourceIndex, TreeModel::LevelRole);
    return level.isValid() && m_allowedLevels.contains(level.toInt());
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
