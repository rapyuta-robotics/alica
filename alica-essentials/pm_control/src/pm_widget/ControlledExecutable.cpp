/*
 * ControlledExecutable.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: Stephan Opfer
 */

#include <qt5/QtWidgets/QMenu>
#include <process_manager/ExecutableMetaData.h>
#include <process_manager/ProcessCommand.h>
#include <process_manager/RobotExecutableRegistry.h>
#include <SystemConfig.h>

#include "ui_ProcessWidget.h"
#include "ui_RobotProcessesWidget.h"

#include "pm_widget/ControlledProcessManager.h"
#include "pm_widget/ControlledExecutable.h"
#include "pm_widget/ControlledRobot.h"

namespace pm_widget {

const string ControlledExecutable::redBackground = "background-color:#FF4719;";
const string ControlledExecutable::greenBackground = "background-color:#66FF66;";
const string ControlledExecutable::grayBackground = "background-color:gray;";

ControlledExecutable::ControlledExecutable(supplementary::ExecutableMetaData* metaExec, ControlledRobot* parentRobot)
        : metaExec(metaExec)
        , memory(0)
        , state('U')
        , cpu(0)
        , _processWidget(new Ui::ProcessWidget())
        , processWidget(new QWidget())
        , parentRobot(parentRobot)
        , runningParamSet(supplementary::ExecutableMetaData::UNKNOWN_PARAMS)
        , desiredParamSet(INT_MAX)
        , publishing(false) {
    for (auto paramEntry : this->metaExec->parameterMap) {
        if (this->desiredParamSet > paramEntry.first) {
            this->desiredParamSet = paramEntry.first;
        }
    }

    this->_processWidget->setupUi(this->processWidget);
    this->_processWidget->processName->setText(QString(this->metaExec->name.c_str()));
    if (this->metaExec->name == "roscore") {
        this->_processWidget->checkBox->setEnabled(false);
        this->_processWidget->checkBox->setChecked(true);
    } else {
        QObject::connect(this->_processWidget->checkBox, SIGNAL(stateChanged(int)), this,
                SLOT(handleCheckBoxStateChanged(int)), Qt::DirectConnection);
    }
    this->processWidget->setContextMenuPolicy(Qt::ContextMenuPolicy::CustomContextMenu);
    connect(this->processWidget, SIGNAL(customContextMenuRequested(const QPoint&)), this,
            SLOT(showContextMenu(const QPoint&)));

    this->msgTimeOut = chrono::duration<double>(
            (*supplementary::SystemConfig::getInstance())["ProcessManaging"]->get<unsigned long>(
                    "PMControl.timeLastMsgReceivedTimeOut", NULL));

    this->pmRegistry = supplementary::RobotExecutableRegistry::get();

    this->parentRobot->addExec(processWidget);
    this->processWidget->show();
}

ControlledExecutable::~ControlledExecutable() {}

void ControlledExecutable::showContextMenu(const QPoint& pos) {
    /* HINT: remember, if there are some problems that way:
     * For QAbstractScrollArea and derived classes you would use:
     * QPoint globalPos = myWidget->viewport()->mapToGlobal(pos); */

    QPoint globalPos = this->processWidget->mapToGlobal(pos);

    QMenu myMenu;
    myMenu.addAction("Start publishing logs");
    myMenu.addAction("Stop publishing logs");

    QAction* selectedItem = myMenu.exec(globalPos);
    if (selectedItem) {
        string selectedString = selectedItem->iconText().toStdString();
        if (selectedString == "Start publishing logs") {
            this->sendProcessCommand(process_manager::ProcessCommand::START_LOG_PUBLISHING);
        } else if (selectedString == "Stop publishing logs") {
            this->sendProcessCommand(process_manager::ProcessCommand::STOP_LOG_PUBLISHING);
        } else {
            cerr << "CE: Chosen context menu option is unhandled!" << endl;
        }
    } else {
        cout << "CE: Nothing chosen in context menu!" << endl;
    }
}

/**
 * Updates the informations about the process with the given information.
 * @param ps The given information about the process.
 */
void ControlledExecutable::handleStat(
        chrono::system_clock::time_point timeMsgReceived, process_manager::ProcessStat ps) {
    this->timeLastMsgReceived = timeMsgReceived;
    this->cpu = ps.cpu;
    this->memory = ps.mem;
    this->state = ps.state;
    this->runningParamSet = ps.paramSet;
    if (ps.publishing == process_manager::ProcessStat::PUBLISHING_ON) {
        this->publishing = true;
    } else if (ps.publishing == process_manager::ProcessStat::PUBLISHING_OFF) {
        this->publishing = false;
    } else {
        cerr << "CE: Unknown publishing flag for process " << this->metaExec->name << endl;
    }
    auto entry = this->metaExec->parameterMap.find(this->runningParamSet);
    if (entry != this->metaExec->parameterMap.end()) {
        stringstream ss;
        ss << "Command: ";
        for (auto param : entry->second) {
            if (param != nullptr) {
                ss << param << " ";
            }
        }
        this->processWidget->setToolTip(QString(ss.str().c_str()));
    } else {
        this->processWidget->setToolTip(QString("Command: Unknown"));
    }
}

void ControlledExecutable::updateGUI(chrono::system_clock::time_point now) {
    if ((now - this->timeLastMsgReceived) >
            this->msgTimeOut) {  // time is over, set controlled executable to not running

        this->_processWidget->processName->setText(QString(this->metaExec->name.c_str()));
        this->_processWidget->cpuState->setText(QString("C: -- %"));
        this->_processWidget->memState->setText(QString("M: -- MB"));
        this->runningParamSet = supplementary::ExecutableMetaData::UNKNOWN_PARAMS;
        this->processWidget->setToolTip(QString(""));
        this->processWidget->setStyleSheet(redBackground.c_str());
    } else {  // message arrived before timeout, update its GUI
        if (this->publishing) {
            this->_processWidget->processName->setText(QString((this->metaExec->name + " (L)").c_str()));
        } else {
            this->_processWidget->processName->setText(QString(this->metaExec->name.c_str()));
        }
        QString cpuString = "C: " + QString::number(this->cpu) + " %";
        QString memString = "M: " + QString::number(this->memory) + " MB";
        this->_processWidget->cpuState->setText(cpuString);
        this->_processWidget->memState->setText(memString);

        switch (this->state) {
            case 'R':  // running
            case 'S':  // interruptable sleeping
            case 'D':  // uninterruptable sleeping
            case 'W':  // paging
                this->processWidget->setStyleSheet(greenBackground.c_str());
                break;
            case 'Z':  // zombie
            case 'T':  // traced, or stopped
                this->processWidget->setStyleSheet(redBackground.c_str());
                this->processWidget->setToolTip(QString(""));
                this->runningParamSet = supplementary::ExecutableMetaData::UNKNOWN_PARAMS;
                break;
            case 'U':
            default:
                cout << "ControlledExec: Unknown process state '" << this->state << "' encountered!" << endl;
                this->processWidget->setStyleSheet(grayBackground.c_str());
                break;
        }
    }
}

void ControlledExecutable::handleBundleComboBoxChanged(QString bundle) {
    for (auto paramEntry : this->metaExec->parameterMap) {
        if (this->desiredParamSet > paramEntry.first) {
            this->desiredParamSet = paramEntry.first;
        }
    }

    if (bundle == "ALL") {
        this->processWidget->show();
        if (this->metaExec->name != "roscore") {
            this->_processWidget->checkBox->setEnabled(true);
            return;
        }
    }

    if (bundle == "RUNNING") {
        switch (this->state) {
            case 'R':  // running
            case 'S':  // interruptable sleeping
            case 'D':  // uninterruptable sleeping
            case 'W':  // paging
            case 'Z':  // zombie
                this->processWidget->show();
                if (this->metaExec->name != "roscore") {
                    this->_processWidget->checkBox->setEnabled(true);
                }
                break;
            case 'T':  // traced, or stopped
            case 'U':  // unknown
            default:
                this->processWidget->hide();
                break;
        }
        return;
    }

    auto bundleMapEntry = this->pmRegistry->getBundlesMap()->find(bundle.toStdString());
    if (bundleMapEntry != this->pmRegistry->getBundlesMap()->end()) {
        bool found = false;
        for (auto processParamSetPair : bundleMapEntry->second) {
            if (this->metaExec->id == processParamSetPair.first) {
                found = true;
                this->desiredParamSet = processParamSetPair.second;
                if (processParamSetPair.second == this->runningParamSet ||
                        this->runningParamSet == supplementary::ExecutableMetaData::UNKNOWN_PARAMS) {
                    if (this->metaExec->name != "roscore") {
                        this->_processWidget->checkBox->setEnabled(true);
                    }
                } else {  // disable the checkbox, if the wrong bundle is selected
                    this->_processWidget->checkBox->setEnabled(false);
                }
            }
        }
        if (found) {
            this->processWidget->show();
        } else {
            this->processWidget->hide();
        }
    } else {
        cerr << "ControlledExecutable: Selected bundle " << bundle.toStdString() << " not found!" << endl;
    }
}

void ControlledExecutable::handleCheckBoxStateChanged(int newState) {
    switch (newState) {
        case Qt::CheckState::Checked:
            this->sendProcessCommand(process_manager::ProcessCommand::START);
            break;
        case Qt::CheckState::Unchecked:
            this->sendProcessCommand(process_manager::ProcessCommand::STOP);
            break;
        case Qt::CheckState::PartiallyChecked:
            cerr << "PMControl: What does it mean, that a process is partially checked?!" << endl;
            break;
        default:
            cerr << "PMControl: Unknown new state of a checkbox!" << endl;
    }
}

void ControlledExecutable::sendProcessCommand(int cmd) {
    this->parentRobot->sendProcessCommand(vector<int>{this->metaExec->id}, vector<int>{this->desiredParamSet}, cmd);
}

}  // namespace pm_widget
