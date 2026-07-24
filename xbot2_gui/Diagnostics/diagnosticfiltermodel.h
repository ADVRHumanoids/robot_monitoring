// Copyright (C) 2024 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef DIAGNOSTICFILTERMODEL_H
#define DIAGNOSTICFILTERMODEL_H

#include <QQmlEngine>
#include <QSortFilterProxyModel>

class DiagnosticFilterModel : public QSortFilterProxyModel
{
    Q_OBJECT
    QML_NAMED_ELEMENT(DiagnosticFilterModel)
    Q_PROPERTY(QString filterText READ filterText WRITE setFilterText NOTIFY filterTextChanged)

public:
    explicit DiagnosticFilterModel(QObject *parent = nullptr);

    QString filterText() const;
    void setFilterText(const QString &filterText);

signals:
    void filterTextChanged();

protected:
    bool filterAcceptsRow(int sourceRow, const QModelIndex &sourceParent) const override;

private:
    bool rowMatches(const QModelIndex &sourceIndex) const;
    bool hasAcceptedDescendant(const QModelIndex &sourceIndex) const;

    QString m_filterText;
};

#endif // DIAGNOSTICFILTERMODEL_H
