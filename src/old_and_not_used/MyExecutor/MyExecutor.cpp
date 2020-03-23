/*
 * MyExecutor.cpp
 *
 *  Created on: Oct 16, 2019
 *      Author: dangield
 */

#include <iostream>
#include <thread>
#include "MyExecutor.hpp"

Exec::Exec() {
	next = 0;
}

MyExecutor::MyExecutor(bool& destructionBool) {
	firstExec = 0;
	this->destroy = &destructionBool;
	std::cout << "Executor initialized." << std::endl;
}

MyExecutor::~MyExecutor() {
	std::cout << "Deleting executor." << std::endl;
	// *this->destroy = true;
	Exec *temp = this->firstExec;
	Exec *temp2 = temp->next;
	while (temp2){
		delete temp;
		temp = temp2;
		temp2 = temp->next;
	}
	delete temp;
}

void MyExecutor::addExec(std::mutex& mut, bool& activationBool, std::chrono::milliseconds threadDelay, std::string name = "Exec") {
	std::cout << "Adding new exec...\t";
	Exec *newExec = new Exec;
	newExec->m = &mut;
	newExec->activate = &activationBool;
	newExec->delay = threadDelay;
	newExec->nextActivationTime = std::chrono::high_resolution_clock::now();
	newExec->name = name;

	if (!this->firstExec) {
		this->firstExec = newExec;
	} else {
		Exec *temp = this->firstExec;
		while (temp->next){
			temp = temp->next;
		}
		temp->next = newExec;
	}
	std::cout << "New exec named " << name << " added." << std::endl;
}

void MyExecutor::spin(){
	if (!this->firstExec){
		std::cout << "Nothing to spin.." << std::endl;
		return;
	}
	Exec *temp = this->firstExec;
	while(temp){
		temp->nextActivationTime = std::chrono::high_resolution_clock::now() + temp->delay;
		temp = temp->next;
	}
	while(!*this->destroy){
		if (!temp) {
			temp = this->firstExec;
	        std::this_thread::sleep_for(std::chrono::microseconds(10));
		}
		if (temp->nextActivationTime < std::chrono::high_resolution_clock::now()) {
			temp->m->lock();
			*temp->activate = true;
			temp->m->unlock();
			temp->nextActivationTime = temp->nextActivationTime + temp->delay;
		}
		temp = temp->next;
	}
}

void MyExecutor::list() {
	if (this->firstExec){
		Exec *temp = this->firstExec;
		std::cout << "Name\tActive\tPeriod" << std::endl;
		std::cout << "------------------------------" << std::endl;
		std::cout << temp->name << "\t" << *temp->activate << "\t" << temp->delay.count() << "ms" << std::endl;
		while(temp->next){
			temp = temp->next;
			std::cout << temp->name << "\t" << *temp->activate << "\t" << temp->delay.count() << "ms" << std::endl;
		}
	} else std::cout << "Executor is empty..." << std::endl;
}


