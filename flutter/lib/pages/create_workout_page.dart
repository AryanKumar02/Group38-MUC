import 'package:flutter/material.dart';
import 'live_workout_page.dart';

class CreateWorkoutPage extends StatefulWidget {
  const CreateWorkoutPage({super.key});

  @override
  _CreateWorkoutPageState createState() => _CreateWorkoutPageState();
}

class _CreateWorkoutPageState extends State<CreateWorkoutPage> {
  TextEditingController workoutNameController = TextEditingController();
  List<String> allExercises = [
    "Bench Press", "Dumbbell Curl", "Squats", "Deadlift",
    "Shoulder Press", "Lunges", "Pull-Ups", "Calf Raises", "Seated Rows", "Step Count"
  ];
  List<Map<String, dynamic>> selectedExercises = [];

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text("Create Workout")),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text("Workout Name:", style: TextStyle(fontSize: 22, fontWeight: FontWeight.bold)),
            TextField(
              controller: workoutNameController,
              decoration: InputDecoration(
                hintText: "Enter workout name",
                border: OutlineInputBorder(),
              ),
            ),
            SizedBox(height: 20),
            Text("Select Exercises:", style: TextStyle(fontSize: 22, fontWeight: FontWeight.bold)),
            Expanded(
              child: ListView.builder(
                itemCount: allExercises.length,
                itemBuilder: (context, index) {
                  return Column(
                    children: [
                      CheckboxListTile(
                        title: Text(allExercises[index]),
                        value: selectedExercises.any((exercise) => exercise['name'] == allExercises[index]),
                        onChanged: (bool? value) {
                          setState(() {
                            if (value == true) {
                              selectedExercises.add({
                                'name': allExercises[index],
                                'sets': 3,
                                'reps': 10
                              });
                            } else {
                              selectedExercises.removeWhere((exercise) => exercise['name'] == allExercises[index]);
                            }
                          });
                        },
                      ),
                      if (selectedExercises.any((exercise) => exercise['name'] == allExercises[index]))
                        Padding(
                          padding: const EdgeInsets.symmetric(horizontal: 16.0),
                          child: Row(
                            mainAxisAlignment: MainAxisAlignment.spaceBetween,
                            children: [
                              Column(
                                children: [
                                  Text("Sets"),
                                  DropdownButton<int>(
                                    value: selectedExercises.firstWhere((exercise) => exercise['name'] == allExercises[index])['sets'],
                                    items: List.generate(5, (i) => i + 1).map((int value) {
                                      return DropdownMenuItem<int>(
                                        value: value,
                                        child: Text(value.toString()),
                                      );
                                    }).toList(),
                                    onChanged: (int? newValue) {
                                      setState(() {
                                        selectedExercises.firstWhere((exercise) => exercise['name'] == allExercises[index])['sets'] = newValue!;
                                      });
                                    },
                                  ),
                                ],
                              ),
                              Column(
                                children: [
                                  Text("Reps"),
                                  DropdownButton<int>(
                                    value: selectedExercises.firstWhere((exercise) => exercise['name'] == allExercises[index])['reps'],
                                    items: List.generate(20, (i) => i + 1).map((int value) {
                                      return DropdownMenuItem<int>(
                                        value: value,
                                        child: Text(value.toString()),
                                      );
                                    }).toList(),
                                    onChanged: (int? newValue) {
                                      setState(() {
                                        selectedExercises.firstWhere((exercise) => exercise['name'] == allExercises[index])['reps'] = newValue!;
                                      });
                                    },
                                  ),
                                ],
                              ),
                            ],
                          ),
                        ),
                    ],
                  );
                },
              ),
            ),
            SizedBox(height: 20),
            Center(
              child: ElevatedButton(
                onPressed: workoutNameController.text.isNotEmpty && selectedExercises.isNotEmpty
                  ? () {
                     final workoutName = workoutNameController.text.trim();
                      Navigator.push(
                        context,
                        MaterialPageRoute(
                          builder: (context) => LiveWorkoutPage(selectedExercises: selectedExercises, sessionName: workoutName),
                        ),
                      );
                    }
                  : null,
                style: ElevatedButton.styleFrom(
                  backgroundColor: Colors.amber,
                  foregroundColor: Colors.white,
                  padding: EdgeInsets.symmetric(horizontal: 32, vertical: 12),
                ),
                child: Text("Start Workout"),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
