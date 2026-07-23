// Copyright (C) 2024 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef TREEMODEL_H
#define TREEMODEL_H

#include <QAbstractItemModel>
#include <QByteArray>
#include <QHash>
#include <QModelIndex>
#include <QVariant>
#include <QVariantMap>
#include <QQmlEngine>

class TreeItem;

//! [0]
class TreeModel : public QAbstractItemModel
{
    Q_OBJECT
    QML_NAMED_ELEMENT(TreeModel)

public:
    Q_DISABLE_COPY_MOVE(TreeModel)

    enum DiagnosticRole {
        NameRole = Qt::UserRole + 1,
        PathRole,
        LevelRole,
        MessageRole,
        HardwareIdRole,
        MetricsRole,
        MetricSummaryRole,
        IsLeafRole
    };
    Q_ENUM(DiagnosticRole)

    explicit TreeModel(QObject *parent = nullptr);
    ~TreeModel() override;

    QVariant data(const QModelIndex &index, int role) const override;
    Qt::ItemFlags flags(const QModelIndex &index) const override;
    QVariant headerData(int section, Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const override;
    QModelIndex index(int row, int column,
                      const QModelIndex &parent = {}) const override;
    QModelIndex parent(const QModelIndex &index) const override;
    int rowCount(const QModelIndex &parent = {}) const override;
    int columnCount(const QModelIndex &parent = {}) const override;
    QHash<int, QByteArray> roleNames() const override;

    Q_INVOKABLE void loadFromDiagnostics(const QVariantMap &diagnostics);

private:
    static std::unique_ptr<TreeItem> createRootItem();
    static void setupModelData(const QVariantMap &diagnostics, TreeItem *parent);

    std::unique_ptr<TreeItem> rootItem;
};
//! [0]

#endif // TREEMODEL_H
