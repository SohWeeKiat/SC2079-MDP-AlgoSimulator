#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_MainForm.h"
#include "../MDPAlgo/FieldObjects.hpp"
#include <vector>
#include <QTimer>

class MainForm : public QMainWindow
{
	Q_OBJECT

public:
	MainForm(QWidget *parent = nullptr);
	~MainForm();

private:
	Ui::MainFormClass ui;
	QTimer* animationTimer;

	QPoint GridSize;
	QPushButton* btnLayout[20][20];
	std::vector<MDP::SFieldObject> field_objects;
	int CurResultTick;
	std::vector<MDP::ObjectState> result;

	bool eventFilter(QObject* obj, QEvent* event) override;

	void RedrawGridButtons();

	POINT GetButtonLocation(QPushButton* btn);
	MDP::SFieldObject GetObjectByLocation(const POINT& loc);
	MDP::SFieldObject GetRobot();

	void OnGridButtonClicked();
	void OnResetObstaclesClicked();
	void OnCalculateClicked();
	void OnAnimateChecked();
	void OnAnimateTick();

	void OnConfigChanges();
	void LoadConfigs();

	void OnSaveObstacles();
	void OnLoadObstacles();
};
