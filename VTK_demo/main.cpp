#include "vtkAutoInit.h" 
VTK_MODULE_INIT(vtkRenderingOpenGL2); // VTK was built with vtkRenderingOpenGL2
VTK_MODULE_INIT(vtkInteractionStyle);

#include <cmath>

#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPolyData.h>
#include <vtkSTLReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkSphereSource.h>
#include <vtkRendererCollection.h>
#include <vtkCellArray.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkPlaneSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPropPicker.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTree.h>
#include <vtkMutableDirectedGraph.h>
#include <vtkGraphLayoutView.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkGraphToPolyData.h>
#include <vtkProperty.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkDijkstraGraphGeodesicPath.h>
#include <vtkCellPicker.h>
#include <vtkCubeSource.h>

#include "config.h"


// Handle mouse events
class MouseInteractorStyle2 : public vtkInteractorStyleTrackballCamera
{
public:
	static MouseInteractorStyle2* New();
	MouseInteractorStyle2() {
		;
	}
	vtkTypeMacro(MouseInteractorStyle2, vtkInteractorStyleTrackballCamera);

	virtual void OnLeftButtonDown()
	{
		int* clickPos = this->GetInteractor()->GetEventPosition();

		// Pick from this location.
		//vtkSmartPointer<vtkPropPicker>  picker =
		//	vtkSmartPointer<vtkPropPicker>::New();
		//picker->Pick(clickPos[0], clickPos[1], 0, this->GetDefaultRenderer());

		// this picker gets ID
		vtkSmartPointer<vtkCellPicker> cell_picker =
			vtkSmartPointer<vtkCellPicker>::New();
		cell_picker->SetTolerance(0.0005);

		//double* pos = picker->GetPickPosition();
		//std::cout << "Pick position (world coordinates) is: "
		//	<< pos[0] << " " << pos[1]
		//	<< " " << pos[2] << std::endl;

		cell_picker->Pick(clickPos[0], clickPos[1], 0, this->GetDefaultRenderer());
		int cell_id = cell_picker->GetCellId();
		if (cell_id != -1) { // picked a point on the mesh
			vtkIdType npts;
			vtkIdType* pts;
			float new_pair_dist = INT_MAX;

			tmpMesh->GetCellPoints(cell_id, npts, pts);
			if (npts < 1) {
				std::cout << "Wrong cell_id with no points! Exiting!" << endl;
				exit(2);
			}
			double pos[3];
			tmpMesh->GetPoint(pts[0], pos);

			if (clicked_point.size() == 0) {
				clicked_point.push_back(pts[0]);
			}

			if (clicked_point.size() >= 1) {
				double pos_old[3];
				tmpMesh->GetPoint(clicked_point[clicked_point.size()-1], pos_old);
				
				new_pair_dist = pow(pos_old[0]-pos[0], 2)+ pow(pos_old[1]-pos[1], 2)+ pow(pos_old[2] - pos[2], 2);
				cout << "The distance is " << new_pair_dist << endl;
				if (new_pair_dist <= MAX_TWO_POINT_DIST) { // if the newly clicked point is not too faraway from old one
					clicked_point.push_back(pts[0]);
					if (clicked_point.size() >= 2) {
						calculateNewPath();
					}
				}
				else {
					std::cout << "Clicked point too faraway from the last point, thus not recording this point!" << endl;
				}
			}
			// draw point
			if (clicked_point.size() == 1 || (clicked_point.size() > 1 && new_pair_dist <= MAX_TWO_POINT_DIST)) {
				std::cout << "Drawing sphere!" << endl;
				//Create a sphere
				vtkSmartPointer<vtkSphereSource> sphereSource =
					vtkSmartPointer<vtkSphereSource>::New();

				sphereSource->SetCenter(pos[0], pos[1], pos[2]);
				sphereSource->SetRadius(0.1);

				//Create a mapper and actor
				vtkSmartPointer<vtkPolyDataMapper> mapper =
					vtkSmartPointer<vtkPolyDataMapper>::New();
				mapper->SetInputConnection(sphereSource->GetOutputPort());

				vtkSmartPointer<vtkActor> actor =
					vtkSmartPointer<vtkActor>::New();
				actor->SetMapper(mapper);
				actor->GetProperty()->SetColor(0.0, 1.0, 1.0); // draw color

				this->GetDefaultRenderer()->AddActor(actor);
			}
		}
		this->Interactor->Render();	
	}

	virtual void OnRightButtonDown()
	{
		vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
	}

	virtual void OnRightButtonUp()
	{
		vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
	}

	void SetMeshInput(vtkAlgorithmOutput* a) {
		tmpMeshInput = a;
	}

	void SetMesh(vtkPolyData* a) {
		tmpMesh = a;
	}

private:
	std::vector<int> clicked_point;
	vtkSmartPointer<vtkDijkstraGraphGeodesicPath> dijkstra;
	vtkSmartPointer<vtkPolyDataMapper> pathMapper;
	vtkSmartPointer<vtkActor> pathActor;
	vtkAlgorithmOutput* tmpMeshInput;
	vtkPolyData* tmpMesh;

	// draw new path from this 
	void calculateNewPath() {
		// new renderer for nearest path
		dijkstra = vtkSmartPointer<vtkDijkstraGraphGeodesicPath>::New();
		dijkstra->SetInputConnection(tmpMeshInput);
		dijkstra->SetStartVertex(clicked_point[clicked_point.size()-2]);
		dijkstra->SetEndVertex(clicked_point[clicked_point.size()-1]);
		dijkstra->Update();

		// Create a mapper and actor
		pathMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		pathMapper->SetInputConnection(dijkstra->GetOutputPort());

		pathActor = vtkSmartPointer<vtkActor>::New();
		pathActor->SetMapper(pathMapper);
		pathActor->GetProperty()->SetColor(1, 0, 0); // Red
		pathActor->GetProperty()->SetLineWidth(4);

		this->GetDefaultRenderer()->AddActor(pathActor);
	}
};

vtkStandardNewMacro(MouseInteractorStyle2);


int main(int argc, char *argv[])
{
	if (argc != 2)
	{
		cout << "Required parameters: Filename" << endl;
		return EXIT_FAILURE;
	}

	std::string inputFilename = argv[1];

	vtkSmartPointer<vtkSTLReader> reader =
		vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName(inputFilename.c_str());
	reader->Update();

	// Visualize
	vtkSmartPointer<vtkPolyDataMapper> mapper =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(reader->GetOutputPort());

	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);

	renderer->AddActor(actor);
	renderer->SetBackground(.3, .6, .3); // Background color green

	renderWindow->Render();

	vtkSmartPointer<MouseInteractorStyle2> style =
		vtkSmartPointer<MouseInteractorStyle2>::New();
	style->SetDefaultRenderer(renderer);
	style->SetMeshInput(reader->GetOutputPort());
	style->SetMesh(reader->GetOutput());

	renderWindowInteractor->SetInteractorStyle(style);

	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}