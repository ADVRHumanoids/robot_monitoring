// Copyright (C) 2024 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef TREEITEM_H
#define TREEITEM_H

#include <QString>
#include <QVariant>
#include <QList>
#include <memory>
#include <vector>

//! [0]
class TreeItem
{
public:
    enum Column {
        NameColumn,
        LevelColumn,
        MessageColumn,
        HardwareIdColumn,
        MetricsColumn,
        ColumnCount
    };

    explicit TreeItem(QString name, QString path = {}, TreeItem *parentItem = nullptr);

    TreeItem *appendChild(std::unique_ptr<TreeItem> &&child);

    TreeItem *child(int row);
    const TreeItem *child(int row) const;
    TreeItem *childByName(const QString &name);
    int childCount() const;
    int columnCount() const;
    QVariant data(int column) const;
    QVariant namedData(int role) const;
    int row() const;
    TreeItem *parentItem();

    void setDiagnostics(int level, QString message, QString hardwareId, QVariantList metrics);
    void sortChildrenRecursively();

private:
    QString metricsSummary() const;

    std::vector<std::unique_ptr<TreeItem>> m_childItems;
    QString m_name;
    QString m_path;
    int m_level = -1;
    QString m_message;
    QString m_hardwareId;
    QVariantList m_metrics;
    TreeItem *m_parentItem;
};
//! [0]

#endif // TREEITEM_H
