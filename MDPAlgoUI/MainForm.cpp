#include "MainForm.h"
#include <QPushButton>
#include <QLabel>
#include <QStyle>
#include <QSettings>
#include <QFileDialog>
#include <QMessageBox>

#include "../MDPAlgo/MazeSolver.hpp"
#include "../MDPAlgo/Config.hpp"
#include "../MDPAlgo/Utils.hpp"
#include "../MDPAlgo/Commands.hpp"

#include "DataSerial.hpp"

MainForm::MainForm(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	this->animationTimer = new QTimer(this);

	QString styleSheet =
		"*[gridBtn=\"true\"] {"
		"	background-color: None;"
		"	border: 1px solid black;"
		"}"
		"*[gridBtn=\"true\"][state=\"1\"] {"
		"background-color: lightblue;"
		"}"
		"*[gridBtn=\"true\"][state=\"2\"] {"
		"background-color: lightblue;"
		"border-top: 3px solid red;"
		"}"
		"*[gridBtn=\"true\"][state=\"3\"] {"
		"background-color: lightblue;"
		"border-right: 3px solid red;"
		"}"
		"*[gridBtn=\"true\"][state=\"4\"] {"
		"background-color: lightblue;"
		"border-bottom: 3px solid red;"
		"}"
		"*[gridBtn=\"true\"][state=\"5\"] {"
		"background-color: lightblue;"
		"border-left: 3px solid red;"
		"}"
		"*[gridBtn=\"true\"][state=\"6\"] {"
		"background-color: lightgreen;"
		"}"
		"*[gridBtn=\"true\"][state=\"7\"] {"
		"background-color: yellow;"
		"}"
		"*[gridBtn=\"true\"][state=\"8\"] {"//route
		"background-color: red;"
		"}"
		"*[gridBtn=\"true\"][state=\"0\"]:hover {"
		"background-color: lightgray;"
		"}"
		"*[gridBtn=\"true\"]:pressed {"
		"background-color: darkgray;"
		"}";
	setStyleSheet(styleSheet);

	QSize BtnSize(30, 30);
	this->GridSize = QPoint(20, 20);

	for (int y = 0; y < this->GridSize.y(); y++) {
		QLabel* txt = new QLabel(QString::number(this->GridSize.y() - y - 1), this);
		txt->setAlignment(Qt::AlignCenter);
		this->ui.gridLayout->addWidget(txt, y, 0, Qt::AlignCenter);
	}
	for (int x = 0; x < this->GridSize.x(); x++) {
		for (int y = 0; y < this->GridSize.y(); y++) {
			auto** btnIndex = &this->btnLayout[this->GridSize.y() - y - 1][x];
			(*btnIndex) = new QPushButton(this);
			connect((*btnIndex), &QPushButton::clicked, this, &MainForm::OnGridButtonClicked);
			(*btnIndex)->installEventFilter(this);
			(*btnIndex)->setFixedSize(BtnSize);
			(*btnIndex)->setProperty("gridBtn", true);
			(*btnIndex)->setProperty("state", 0);
			this->ui.gridLayout->addWidget((*btnIndex), y, x + 1, Qt::AlignCenter);
		}
	}
	for (int x = 0; x < this->GridSize.x(); x++) {
		QLabel* txt = new QLabel(QString::number(x), this);
		txt->setAlignment(Qt::AlignCenter);
		this->ui.gridLayout->addWidget(txt, this->GridSize.y(), x + 1, Qt::AlignCenter);
	}
	this->ui.gridLayout->setHorizontalSpacing(0);
	this->ui.gridLayout->setVerticalSpacing(0);
	this->ui.gridLayout->setSpacing(0);
	this->ui.gridLayout->setContentsMargins(0, 0, 0, 0);

	connect(this->ui.bResetObstacles, &QPushButton::clicked, this, &MainForm::OnResetObstaclesClicked);
	connect(this->ui.bResetRobot, &QPushButton::clicked, [this]() {
		auto robot = this->GetRobot();
		if (!robot) return;
		robot->Update({ 1,1 }, MDP::FaceDirection::FD_North);
		this->RedrawGridButtons();
	});

	connect(this->ui.bCalculate, &QPushButton::clicked, this, &MainForm::OnCalculateClicked);
	connect(this->ui.cBAnimatePath, &QCheckBox::stateChanged, this, &MainForm::OnAnimateChecked);
	connect(this->animationTimer, &QTimer::timeout, this, &MainForm::OnAnimateTick);

	/*config changes*/
	connect(this->ui.NUDLeftWheel, &QSpinBox::valueChanged, this, &MainForm::OnConfigChanges);
	connect(this->ui.NUDRightWheel, &QSpinBox::valueChanged, this, &MainForm::OnConfigChanges);
	connect(this->ui.NUDIterations, &QSpinBox::valueChanged, this, &MainForm::OnConfigChanges);
	connect(this->ui.NUDSafeCost, &QSpinBox::valueChanged, this, &MainForm::OnConfigChanges);
	connect(this->ui.NUDExpandedCell, &QSpinBox::valueChanged, this, &MainForm::OnConfigChanges);
	connect(this->ui.cBLimit90FWBW, &QCheckBox::stateChanged, this, &MainForm::OnConfigChanges);
	connect(this->ui.cBOutsideCommands, &QCheckBox::stateChanged, this, &MainForm::OnConfigChanges);

	connect(this->ui.actionLoad_Obstacles, &QAction::triggered, this, &MainForm::OnLoadObstacles);
	connect(this->ui.actionSave_Obstacles, &QAction::triggered, this, &MainForm::OnSaveObstacles);

	connect(this->ui.horizontalSlider, &QSlider::valueChanged, [this](int value) {
		this->animationTimer->setInterval(this->ui.horizontalSlider->value());
	});

	/*initialize default rebot position*/
	this->field_objects.push_back(std::make_shared<MDP::FieldRobot>());
	this->RedrawGridButtons();
	this->LoadConfigs();
}

MainForm::~MainForm()
{}

bool MainForm::eventFilter(QObject* obj, QEvent* event)
{
	for (int x = 0; x < this->GridSize.x(); x++) {
		for (int y = 0; y < this->GridSize.y(); y++) {
			if (obj == (QObject*)this->btnLayout[y][x]) {
				if (event->type() == QEvent::Enter)
				{
					this->ui.statusBar->showMessage(QString("Coord: %1, %2").arg(x).arg(y));
				}
				return QWidget::eventFilter(obj, event);
			}
		}
	}
	return QWidget::eventFilter(obj, event);
}

void MainForm::RedrawGridButtons()
{
	for (int x = 0; x < this->GridSize.x(); x++) 
	{
		for (int y = 0; y < this->GridSize.y(); y++) 
		{
			//reset
			auto* btn = this->btnLayout[this->GridSize.y() - y - 1][x];
			auto prop = btn->property("state");
			if (prop.isValid() && prop.toInt() == 0) continue;

			btn->setProperty("state", 0);
			btn->style()->unpolish(btn);
			btn->style()->polish(btn);

			/*QEvent event(QEvent::StyleChange);
			QApplication::sendEvent(btn, &event);*/
			btn->update();
			btn->updateGeometry();
		}
	}
	for (auto& s : this->result) {
		auto btn = this->btnLayout[s.m_location.y][s.m_location.x];
		btn->setProperty("state", 8);//path
		btn->style()->unpolish(btn);
		btn->style()->polish(btn);

		btn->update();
		btn->updateGeometry();
	}
	for (auto& obj : this->field_objects) {
		auto pts = obj->GetRectPoints();
		for (auto& pt : pts) {
			auto btn = this->btnLayout[pt.y][pt.x];
			btn->setProperty("state",static_cast<int>(obj->GetGridBoxType(pt)));

			btn->style()->unpolish(btn);
			btn->style()->polish(btn);

			/*QEvent event(QEvent::StyleChange);
			QApplication::sendEvent(btn, &event);*/
			btn->update();
			btn->updateGeometry();
		}
	}
}

POINT MainForm::GetButtonLocation(QPushButton* btn)
{
	for (int x = 0; x < this->GridSize.x(); x++)
	{
		for (int y = 0; y < this->GridSize.y(); y++)
		{
			if (this->btnLayout[y][x] == btn) {
				return { x, y };
			}
		}
	}
	return { -1,-1 };
}

MDP::SFieldObject MainForm::GetObjectByLocation(const POINT& loc)
{
	for (auto& obj : this->field_objects) {
		if (obj->IsInRect(loc)) return obj;
	}
	return nullptr;
}

MDP::SFieldObject MainForm::GetRobot()
{
	auto it = std::find_if(this->field_objects.begin(), this->field_objects.end(), [](const MDP::SFieldObject& obj) {
		return std::dynamic_pointer_cast<MDP::FieldRobot>(obj);
	});
	if (it != this->field_objects.end())
		return *it;
	return nullptr;
}

void MainForm::OnGridButtonClicked()
{
	auto button = qobject_cast<QPushButton*>(sender());
	auto loc = this->GetButtonLocation(button);
	if (loc.x == -1 || loc.y == -1) return;
	auto obj = this->GetObjectByLocation(loc);
	if (!obj) {
		//allocate new block + redraw
		auto blk = std::make_shared<MDP::FieldBlock>(loc, MDP::FaceDirection::FD_North, this->field_objects.size());
		this->field_objects.push_back(blk);
		return this->RedrawGridButtons();
	}
	//check is it a robot type
	if (std::dynamic_pointer_cast<MDP::FieldRobot>(obj))
		return;
	//is not robot
	obj->ChangeDirection();
	if (obj->GetDirection() == MDP::FaceDirection::FD_North) {
		//repeated, remove from list
		auto it = std::find(this->field_objects.begin(), this->field_objects.end(), obj);
		if (it != this->field_objects.end())
			this->field_objects.erase(it);
	}
	return this->RedrawGridButtons();
}

void MainForm::OnResetObstaclesClicked()
{
	this->field_objects.erase(std::remove_if(this->field_objects.begin(), this->field_objects.end(), [](const MDP::SFieldObject& obj) {
		return !std::dynamic_pointer_cast<MDP::FieldRobot>(obj);
	}), this->field_objects.end());
	this->RedrawGridButtons();
}

void MainForm::OnCalculateClicked()
{
	auto robot = this->GetRobot();
	if (!robot) return;
	MDP::MazeSolver solver({ this->GridSize.x(), this->GridSize.y() }, robot->GetLoc(), robot->GetDirection());
	for (auto& o : this->field_objects) {
		if (!std::dynamic_pointer_cast<MDP::FieldRobot>(o))
			solver.AddObstacle(o);
	}
	this->result = solver.GetOptimalOrderDP(false);

	this->ui.PathTable->setRowCount(0);
	for (auto& entry : this->result)
	{
		int row = this->ui.PathTable->rowCount();
		this->ui.PathTable->insertRow(row);
		this->ui.PathTable->setItem(row, 0, new QTableWidgetItem(
			QString("%L1,%L2")
			.arg(entry.m_location.x)
			.arg(entry.m_location.y)));

		this->ui.PathTable->setItem(row, 1, new QTableWidgetItem(
			QString::fromStdString(Utils::to_string(entry.m_Fd))));
		this->ui.PathTable->setItem(row, 2, new QTableWidgetItem(
			QString::number(entry.penalty)));
		this->ui.PathTable->setItem(row, 3, new QTableWidgetItem(
			QString::number(entry.snapshot_id)));
	}
	auto commands = MDP::command_generator(this->result, solver.GetObstacles());
	this->ui.CommandList->setRowCount(0);
	for (auto& c : commands)
	{
		int row = this->ui.CommandList->rowCount();
		this->ui.CommandList->insertRow(row);
		this->ui.CommandList->setItem(row, 0, new QTableWidgetItem(
			QString("%L1,%L2")
			.arg(c.GetState().m_location.x)
			.arg(c.GetState().m_location.y)));

		this->ui.CommandList->setItem(row, 1, new QTableWidgetItem(
			QString::fromStdString(Utils::to_string(c.GetState().m_Fd))));
		this->ui.CommandList->setItem(row, 2, new QTableWidgetItem(
			QString::fromStdString(c.ToString())));
	}

	this->CurResultTick = 0;
	this->RedrawGridButtons();
}

void MainForm::OnAnimateChecked()
{
	if (this->ui.cBAnimatePath->isChecked())
		this->animationTimer->start(this->ui.horizontalSlider->value());
	else {
		this->animationTimer->stop();
		if (this->result.empty()) return;
		this->CurResultTick = 0;
		auto& state = this->result[0];
		auto robot = this->GetRobot();
		if (!robot) return;
		robot->Update(state.m_location, state.m_Fd);
		this->RedrawGridButtons();
	}
}

void MainForm::OnAnimateTick()
{
	if (this->CurResultTick < 0 || this->CurResultTick >= this->result.size())
		this->CurResultTick = 0;
	if (this->result.empty())
		return;
	auto robot = this->GetRobot();
	if (!robot) return;
	auto& curState = this->result[this->CurResultTick++];
	robot->Update(curState.m_location, curState.m_Fd);
	this->RedrawGridButtons();
}

void MainForm::OnConfigChanges()
{
	MDP::Config::get().Set_LEFTWHEEL(this->ui.NUDLeftWheel->value());
	MDP::Config::get().Set_RIGHTWHEEL(this->ui.NUDRightWheel->value());
	MDP::Config::get().Set_ITERATIONS(this->ui.NUDIterations->value());
	MDP::Config::get().Set_SAFE_COST(this->ui.NUDSafeCost->value());
	MDP::Config::get().Set_EXPANDED_CELL(this->ui.NUDExpandedCell->value());
	MDP::Config::get().Set_LimitMax90(this->ui.cBLimit90FWBW->isChecked());
	MDP::Config::get().Set_OutsideCommand(this->ui.cBOutsideCommands->isChecked());

	QSettings setting("settings.ini", QSettings::Format::IniFormat);
	setting.beginGroup("config");
	setting.setValue("LeftWheel", MDP::Config::get().Get_LEFTWHEEL()); 
	setting.setValue("RightWheel", MDP::Config::get().Get_RIGHTWHEEL());
	setting.setValue("Iterations", MDP::Config::get().Get_ITERATIONS());
	setting.setValue("SafeCost", MDP::Config::get().Get_SAFE_COST());
	setting.setValue("ExpandedCell", MDP::Config::get().Get_EXPANDED_CELL());
	setting.setValue("LimitMax90", MDP::Config::get().Is_LimitMax90());
	setting.setValue("OutsideCommand", MDP::Config::get().Is_OutsideCommand());
	setting.endGroup();
	setting.sync();
}

void SetNUD(QSpinBox* box, int value)
{
	box->blockSignals(true);
	box->setValue(value);
	box->blockSignals(false);
}

void SetcB(QCheckBox* box, bool value)
{
	box->blockSignals(true);
	box->setChecked(value);
	box->blockSignals(false);
}

void MainForm::LoadConfigs()
{
	QSettings setting("settings.ini", QSettings::Format::IniFormat);
	MDP::Config::get().Set_LEFTWHEEL(setting.value("config/LeftWheel", MDP::Config::get().Get_LEFTWHEEL()).toInt());
	MDP::Config::get().Set_RIGHTWHEEL(setting.value("config/RightWheel", MDP::Config::get().Get_RIGHTWHEEL()).toInt());
	MDP::Config::get().Set_ITERATIONS(setting.value("config/Iterations", MDP::Config::get().Get_ITERATIONS()).toInt());
	MDP::Config::get().Set_SAFE_COST(setting.value("config/SafeCost", MDP::Config::get().Get_SAFE_COST()).toInt());
	MDP::Config::get().Set_EXPANDED_CELL(setting.value("config/ExpandedCell", MDP::Config::get().Get_EXPANDED_CELL()).toInt());
	MDP::Config::get().Set_LimitMax90(setting.value("config/LimitMax90", MDP::Config::get().Is_LimitMax90()).toBool());
	MDP::Config::get().Set_OutsideCommand(setting.value("config/OutsideCommand", MDP::Config::get().Is_OutsideCommand()).toBool());

	SetNUD(this->ui.NUDLeftWheel,MDP::Config::get().Get_LEFTWHEEL());
	SetNUD(this->ui.NUDRightWheel,MDP::Config::get().Get_RIGHTWHEEL());
	SetNUD(this->ui.NUDIterations,MDP::Config::get().Get_ITERATIONS());
	SetNUD(this->ui.NUDSafeCost,MDP::Config::get().Get_SAFE_COST());
	SetNUD(this->ui.NUDExpandedCell,MDP::Config::get().Get_EXPANDED_CELL());

	SetcB(this->ui.cBLimit90FWBW, MDP::Config::get().Is_LimitMax90());
	SetcB(this->ui.cBOutsideCommands, MDP::Config::get().Is_OutsideCommand());
}

void MainForm::OnSaveObstacles()
{
	auto file_path = QFileDialog::getSaveFileName(this, "Save MDP File", qApp->applicationDirPath(),
		"mdp files (*.mdp)");
	if (file_path.isEmpty()) return;
	QFile file(file_path);
	if (!file.open(QIODevice::ReadWrite)) {
		QMessageBox::warning(this, "Save", "Failed to save mdp file");
		return;
	}
	QDataStream stream(&file);
	int count = this->field_objects.size() - 1;
	stream << count;
	for (auto& o : this->field_objects) {
		if (std::dynamic_pointer_cast<MDP::FieldRobot>(o))
			continue;
		stream << o;
	}
}

void MainForm::OnLoadObstacles()
{
	auto file_path = QFileDialog::getOpenFileName(this, "Select MDP File", qApp->applicationDirPath(),
		"mdp files (*.mdp)");
	if (file_path.isEmpty()) return;
	QFile file(file_path);
	if (!file.open(QIODevice::ReadOnly)) {
		QMessageBox::warning(this, "Load", "Failed to load mdp file");
		return;
	}
	QDataStream stream(&file);
	this->OnResetObstaclesClicked();
	int count = 0;
	stream >> count;
	for (int i = 0; i < count; i++) {
		auto obj = std::make_shared<MDP::FieldBlock>(POINT{ 1,1 });
		
		int snapshot_id = -1;
		int x = 0, y = 0;
		MDP::FaceDirection fd;

		stream >> snapshot_id;
		stream >> x;
		stream >> y;
		stream >> fd;

		obj->UpdateSnapshotID(snapshot_id);
		obj->Update(POINT{ x,y }, fd);
		this->field_objects.push_back(obj);
	}
	this->RedrawGridButtons();
}