// Copyright (C) 2024 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef DIAGNOSTICFILTERMODEL_H
#define DIAGNOSTICFILTERMODEL_H

#include <QQmlEngine>
#include <QSortFilterProxyModel>
#include <QStringList>

class DiagnosticFilterModel : public QSortFilterProxyModel
{
    Q_OBJECT
    QML_NAMED_ELEMENT(DiagnosticFilterModel)
    Q_PROPERTY(QString filterText READ filterText WRITE setFilterText NOTIFY filterTextChanged)
    Q_PROPERTY(int minimumLevel READ minimumLevel WRITE setMinimumLevel NOTIFY minimumLevelChanged)

public:
    explicit DiagnosticFilterModel(QObject *parent = nullptr);

    QString filterText() const;
    void setFilterText(const QString &filterText);
    int minimumLevel() const;
    void setMinimumLevel(int minimumLevel);
    Q_INVOKABLE QModelIndex indexForPath(const QString &path, int column = 0) const;
    Q_INVOKABLE QStringList expandablePaths() const;

signals:
    void filterTextChanged();
    void minimumLevelChanged();

protected:
    bool filterAcceptsRow(int sourceRow, const QModelIndex &sourceParent) const override;

private:
    bool rowMatches(const QModelIndex &sourceIndex) const;
    bool textMatches(const QModelIndex &sourceIndex) const;
    bool levelMatches(const QModelIndex &sourceIndex) const;
    bool hasAcceptedDescendant(const QModelIndex &sourceIndex) const;

    QString m_filterText;
    int m_minimumLevel = -1;
};

#endif // DIAGNOSTICFILTERMODEL_H
