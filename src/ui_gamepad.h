/********************************************************************************
** Form generated from reading UI file 'gamepad.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GAMEPAD_H
#define UI_GAMEPAD_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_gamepad
{
public:
    QWidget *dockWidgetContents;
    QGridLayout *gridLayout;
    QSpacerItem *verticalSpacer;
    QPushButton *pushButton_2;

    void setupUi(QDockWidget *gamepad)
    {
        if (gamepad->objectName().isEmpty())
            gamepad->setObjectName(QStringLiteral("gamepad"));
        gamepad->resize(403, 652);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QStringLiteral("dockWidgetContents"));
        gridLayout = new QGridLayout(dockWidgetContents);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer, 2, 0, 1, 1);

        pushButton_2 = new QPushButton(dockWidgetContents);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));

        gridLayout->addWidget(pushButton_2, 1, 0, 1, 1);

        gamepad->setWidget(dockWidgetContents);

        retranslateUi(gamepad);

        QMetaObject::connectSlotsByName(gamepad);
    } // setupUi

    void retranslateUi(QDockWidget *gamepad)
    {
        gamepad->setWindowTitle(QApplication::translate("gamepad", "Controller", 0));
        pushButton_2->setText(QApplication::translate("gamepad", "Connect", 0));
    } // retranslateUi

};

namespace Ui {
    class gamepad: public Ui_gamepad {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GAMEPAD_H
