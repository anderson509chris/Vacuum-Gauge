#ifndef VacuumGuage_H_
#define VacuumGuage_H_

bool saveData(void);
bool loadData(void);
void displayPressure(float pressure);
bool readPressure(void *);
bool saveChanges(void *);
void setup(void);
void loop(void);

#endif

