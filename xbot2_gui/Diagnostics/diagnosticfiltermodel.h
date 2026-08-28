// Copyright (C) 2024 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef DIAGNOSTICFILTERMODEL_H
#define DIAGNOSTICFILTERMODEL_H

#include <QQmlEngine>
#include <QSortFilterProxyModel>
#include <QStringList>
#include <QVariantList>

class DiagnosticFilterModel : public QSortFilterProxyModel
{
    Q_OBJECT
    QML_NAMED_ELEMENT(DiagnosticFilterModel)
    Q_PROPERTY(QString filterText READ filterText WRITE setFilterText NOTIFY filterTextChanged)
    Q_PROPERTY(QVariantList allowedLevels READ allowedLevels WRITE setAllowedLevels NOTIFY allowedLevelsChanged)
    Q_PROPERTY(int minimumLevel READ minimumLevel WRITE setMinimumLevel NOTIFY minimumLevelChanged)

public:
    explicit DiagnosticFilterModel(QObject *parent = nullptr);

    QString filterText() const;
    void setFilterText(const QString &filterText);
    QVariantList allowedLevels() const;
    void setAllowedLevels(const QVariantList &allowedLevels);
    int minimumLevel() const;
    void setMinimumLevel(int minimumLevel);
    Q_INVOKABLE bool isLevelEnabled(int level) const;
    Q_INVOKABLE void enableLevel(int level, bool enabled);
    Q_INVOKABLE QModelIndex indexForPath(const QString &path, int column = 0) const;
    Q_INVOKABLE QStringList expandablePaths() const;

signals:
    void filterTextChanged();
    void allowedLevelsChanged();
    void minimumLevelChanged();

protected:
    bool filterAcceptsRow(int sourceRow, const QModelIndex &sourceParent) const override;

private:
    bool rowMatches(const QModelIndex &sourceIndex) const;
    bool textMatches(const QModelIndex &sourceIndex) const;
    bool levelMatches(const QModelIndex &sourceIndex) const;
    bool hasAcceptedDescendant(const QModelIndex &sourceIndex) const;

    QString m_filterText;
    QVariantList m_allowedLevels {0, 1, 2, 3};
    int m_minimumLevel = -1;
};

#endif // DIAGNOSTICFILTERMODEL_H
