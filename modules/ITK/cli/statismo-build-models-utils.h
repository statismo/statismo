#include <string>
#include <vector>
#include <map>
#include <set>
#include <iostream>

using namespace std;

typedef list<string> StringList;
StringList getFileList(std::string path);

template<class MeshType>
typename MeshType::Pointer calculateMeanMesh(vector<typename MeshType::Pointer> meshes);

template<class MeshType>
float calculateMeshDistance(typename MeshType::Pointer mesh1, typename MeshType::Pointer mesh2);

template<class MeshType, class LandmarkBasedTransformInitializerType, class TransformType, class FilterType>
vector<typename MeshType::Pointer> superimposeMeshes(vector<typename MeshType::Pointer> originalMeshes, typename MeshType::Pointer referenceMesh, std::set<unsigned> landmarkIndices);

template<class MeshType, class LandmarkBasedTransformInitializerType, class TransformType, class FilterType>
typename MeshType::Pointer calculateProcrustesMeanMesh(vector<typename MeshType::Pointer> meshes, unsigned maxIterations, unsigned nrOfLandmarks, float breakIfChangeBelow);



template<class MeshType, class LandmarkBasedTransformInitializerType, class TransformType, class FilterType>
typename MeshType::Pointer calculateProcrustesMeanMesh(vector<typename MeshType::Pointer> meshes, unsigned maxIterations, unsigned nrOfLandmarks, float breakIfChangeBelow) {
    //the initial mesh to which all others will be aligned to is the first one in the list here. Any other mesh could be chosen as well
    typename MeshType::Pointer referenceMesh = *meshes.begin();

    unsigned rngSeed = time(0);
    unsigned meshVerticesCount = referenceMesh->GetNumberOfPoints();
    srand(rngSeed);
    std::set<unsigned> pointNumbers;
    while (pointNumbers.size() < min(nrOfLandmarks, meshVerticesCount)) {
        unsigned randomIndex = ((unsigned) rand()) % meshVerticesCount;
        pointNumbers.insert(randomIndex);
    }

    float fPreviousDifference = -1;

    for (unsigned i = 0; i < maxIterations; i++) {
        //calculate the difference to the previous iteration's mesh and break if the difference is very small
        vector<typename MeshType::Pointer> translatedMeshes = superimposeMeshes<MeshType, LandmarkBasedTransformInitializerType, TransformType, FilterType>(meshes, referenceMesh, pointNumbers);
        typename MeshType::Pointer meanMesh = calculateMeanMesh<MeshType>(translatedMeshes);
        float fDifference = calculateMeshDistance<MeshType>(meanMesh, referenceMesh);
        float fDifferenceDelta = abs(fDifference - fPreviousDifference);
        fPreviousDifference = fDifference;
        referenceMesh = meanMesh;

        if (fDifferenceDelta < breakIfChangeBelow) {
            break;
        }
    }
    return referenceMesh;
}

template<class MeshType, class LandmarkBasedTransformInitializerType, class TransformType, class FilterType>
vector<typename MeshType::Pointer> superimposeMeshes(vector<typename MeshType::Pointer> originalMeshes, typename MeshType::Pointer referenceMesh, std::set<unsigned> landmarkIndices) {
    vector<typename MeshType::Pointer> translatedMeshes(originalMeshes.begin(), originalMeshes.end());
    for (typename vector<typename MeshType::Pointer>::iterator it = translatedMeshes.begin(); it != translatedMeshes.end(); it++) {
        typedef typename LandmarkBasedTransformInitializerType::LandmarkPointContainer LandmarkContainerType;
        LandmarkContainerType movingLandmarks;
        LandmarkContainerType fixedLandmarks;
        typename MeshType::Pointer movingMesh = *it;

        //Only use a subset of the meshes' points for the alignment since we don't have that many degrees of freedom anyways and since calculating a SVD with too many points is expensive
        for (std::set<unsigned>::const_iterator rng = landmarkIndices.begin(); rng != landmarkIndices.end(); rng++) {
            movingLandmarks.push_back(movingMesh->GetPoint(*rng));
            fixedLandmarks.push_back(referenceMesh->GetPoint(*rng));
        }

        //only rotate & translate the moving mesh to best fit with the fixed mesh; there's no scaling taking place.
        typename LandmarkBasedTransformInitializerType::Pointer landmarkBasedTransformInitializer = LandmarkBasedTransformInitializerType::New();
        landmarkBasedTransformInitializer->SetFixedLandmarks(fixedLandmarks);
        landmarkBasedTransformInitializer->SetMovingLandmarks(movingLandmarks);
        typename TransformType::Pointer transform = TransformType::New();
        transform->SetIdentity();
        landmarkBasedTransformInitializer->SetTransform(transform);
        landmarkBasedTransformInitializer->InitializeTransform();

        typename FilterType::Pointer filter = FilterType::New();
        filter->SetInput(movingMesh);
        filter->SetTransform(transform);
        filter->Update();

        *it = filter->GetOutput();
    }
    return translatedMeshes;
}


template<class MeshType>
float calculateMeshDistance(typename MeshType::Pointer mesh1, typename MeshType::Pointer mesh2) {
    if (mesh1->GetNumberOfPoints() != mesh2->GetNumberOfPoints() || mesh1->GetNumberOfCells() != mesh2->GetNumberOfCells()) {
        itk::ExceptionObject e(__FILE__, __LINE__, "Both meshes must have the same number of Edges & Vertices", ITK_LOCATION);
        throw e;
    }

    float fDifference = 0;
    for (unsigned j = 0; j < mesh1->GetNumberOfPoints(); j++) {
        typename MeshType::PointType point1 = mesh1->GetPoint(j);
        typename MeshType::PointType point2 = mesh2->GetPoint(j);
        for (unsigned k = 0; k < mesh1->GetPoint(j).Size(); k++) {
            fDifference += (point1[k] - point2[k])*(point1[k] - point2[k]);
        }
    }
    fDifference /= (mesh1->GetNumberOfPoints() + MeshType::PointDimension);
    return fDifference;
}

template<class MeshType>
typename MeshType::Pointer calculateMeanMesh(vector<typename MeshType::Pointer> meshes) {
    if (meshes.size() == 0) {
        itk::ExceptionObject e(__FILE__, __LINE__, "Can't calculate the mean since no meshes were provided.", ITK_LOCATION);
        throw e;
    }

    typename MeshType::Pointer meanMesh;
    for (typename vector<typename MeshType::Pointer>::const_iterator i = meshes.begin(); i != meshes.end(); i++) {
        typename MeshType::Pointer mesh = *i;
        if (!meanMesh) {
            meanMesh = mesh;
        } else {
            if (meanMesh->GetNumberOfPoints() != mesh->GetNumberOfPoints() || meanMesh->GetNumberOfCells() != mesh->GetNumberOfCells()) {
                itk::ExceptionObject e(__FILE__, __LINE__, "All meshes must have the same number of Edges & Vertices", ITK_LOCATION);
                throw e;
            }

            //sum up all meshes
            for (unsigned j = 0; j < meanMesh->GetNumberOfPoints(); j++) {
                typename MeshType::PointType point = meanMesh->GetPoint(j);
                typename MeshType::PointType pointToAdd = mesh->GetPoint(j);
                for (unsigned k = 0; k < meanMesh->GetPoint(j).Size(); k++) {
                    point[k] = point[k] + pointToAdd[k];
                }
                meanMesh->SetPoint(j, point);
            }
        }
    }

    //divide by the # of meshes
    for (unsigned j = 0; j < meanMesh->GetNumberOfPoints(); j++) {
        typename MeshType::PointType point = meanMesh->GetPoint(j);
        for (unsigned k = 0; k < meanMesh->GetPoint(j).Size(); k++) {
            point[k] /= meshes.size();
        }
        meanMesh->SetPoint(j, point);
    }

    return meanMesh;
}


StringList getFileList(std::string path) {
    StringList fileList;

    ifstream file;
    try {
        file.exceptions(ifstream::failbit | ifstream::badbit);
        file.open(path.c_str(), ifstream::in);
        string line;
        while (getline(file, line)) {
            if (line != "") {
                fileList.push_back(line);
            }
        }
    } catch (ifstream::failure e) {
        if (file.eof() == 0) {
            throw e;
        }
    }
    file.close();
    return fileList;
}