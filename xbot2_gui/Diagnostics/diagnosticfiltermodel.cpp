// Copyright (C) 2024 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "diagnosticfiltermodel.h"
#include "treemodel.h"

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

bool DiagnosticFilterModel::filterAcceptsRow(int sourceRow, const QModelIndex &sourceParent) const
{
    if (m_filterText.isEmpty())
        return true;
    if (!sourceModel())
        return false;

    const auto sourceIndex = sourceModel()->index(sourceRow, 0, sourceParent);
    return rowMatches(sourceIndex) || hasAcceptedDescendant(sourceIndex);
}

bool DiagnosticFilterModel::rowMatches(const QModelIndex &sourceIndex) const
{
    const auto matches = [this, &sourceIndex](int role) {
        return sourceModel()->data(sourceIndex, role).toString().contains(m_filterText, Qt::CaseInsensitive);
    };

    return matches(TreeModel::NameRole)
           || matches(TreeModel::PathRole)
           || matches(TreeModel::LevelRole)
           || matches(TreeModel::MessageRole)
           || matches(TreeModel::HardwareIdRole)
           || matches(TreeModel::MetricSummaryRole);
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
