#include <itkDirectory.h>
#include <itkMesh.h>
#include <itkMeshFileWriter.h>
#include <itkMeshFileReader.h>
#include <itkLandmarkBasedTransformInitializer.h>
#include <itkRigid3DTransform.h>
#include <itkTransformMeshFilter.h>
#include <itkImage.h>

#include "itkDataManager.h"
#include "itkPCAModelBuilder.h"
#include "itkStandardMeshRepresenter.h"
#include "itkStatisticalModel.h"

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

#include "statismo-build-models-utils.h"

namespace po = boost::program_options;

struct programOptions {
    bool bDisplayHelp;
    string strDataListFile;
    string strProcrustesMode;
    string strProcrustesReferenceFile;
    string strOutputFileName;
    float fNoiseVariance;
    bool bAutoNoise;
};

po::options_description initializeProgramOptions(programOptions& poParameters);
bool isOptionsConflictPresent(programOptions& opt);
void buildAndSaveShapeModel(programOptions opt);



int main(int argc, char** argv) {
    programOptions poParameters;

    po::positional_options_description optPositional;
    optPositional.add("output-file", 1);
    po::options_description optAllOptions = initializeProgramOptions(poParameters);


    po::variables_map vm;
    try {
        po::parsed_options parsedOptions = po::command_line_parser(argc, argv).options(optAllOptions).positional(optPositional).run();
        po::store(parsedOptions, vm);
        po::notify(vm);
    } catch (po::error& e) {
        cerr << "An exception occurred while parsing the Command line:"<<endl;
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    }

    if (poParameters.bDisplayHelp == true) {
        cout << optAllOptions << endl;
        return EXIT_SUCCESS;
    }
    if (isOptionsConflictPresent(poParameters) == true)	{
        cerr << "A conflict in the options exists or insufficient options were set." << endl;
        cout << optAllOptions << endl;
        return EXIT_FAILURE;
    }

    try {
        buildAndSaveShapeModel(poParameters);
    } catch (ifstream::failure & e) {
        cerr << "Could not read the data-list:" << endl;
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    } catch (itk::ExceptionObject & e) {
        cerr << "Could not build the model:" << endl;
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

bool isOptionsConflictPresent(programOptions& opt) {
    boost::algorithm::to_lower(opt.strProcrustesMode);

    if (opt.strProcrustesMode != "reference" && opt.strProcrustesMode != "gpa") {
        return true;
    }

    if(opt.strProcrustesMode == "reference" && opt.strProcrustesReferenceFile == "") {
        return true;
    }

    if (opt.strProcrustesMode == "gpa" && opt.strProcrustesReferenceFile != "") {
        return true;
    }

    if (opt.strDataListFile == "" || opt.strOutputFileName == "") {
        return true;
    }

    if (opt.strDataListFile == opt.strOutputFileName) {
        return true;
    }

    if (opt.strProcrustesMode == "reference") {
        if (opt.strDataListFile == opt.strProcrustesReferenceFile || opt.strOutputFileName == opt.strProcrustesReferenceFile) {
            return true;
        }
    }

    return false;
}

void buildAndSaveShapeModel(programOptions opt) {
    const unsigned Dimensions = 3;
    typedef itk::Mesh<float, Dimensions> MeshType;
    typedef itk::StandardMeshRepresenter<float, Dimensions> RepresenterType;
    typedef itk::PCAModelBuilder<MeshType> PCAModelBuilder;
    typedef itk::StatisticalModel<MeshType> StatisticalModelType;
    typedef itk::DataManager<MeshType> DataManagerType;
    typedef itk::MeshFileReader<MeshType> MeshReaderType;
    typedef list<pair<MeshType::Pointer, string> > MeshList;

    RepresenterType::Pointer representer = RepresenterType::New();
    DataManagerType::Pointer dataManager = DataManagerType::New();

    MeshList meshes;
    StringList filenames = getFileList(opt.strDataListFile);
    for (StringList::const_iterator it = filenames.begin(); it != filenames.end(); it++) {
        MeshReaderType::Pointer reader = MeshReaderType::New();
        reader->SetFileName(it->c_str());
        reader->Update();
        meshes.push_back(make_pair(reader->GetOutput(), *it));
    }

    if (meshes.size() == 0) {
        itk::ExceptionObject e(__FILE__, __LINE__, "The specified data-list is empty.", ITK_LOCATION);
        throw e;
    }

    if (opt.strProcrustesMode == "reference") {
        MeshReaderType::Pointer refReader = MeshReaderType::New();
        refReader->SetFileName(opt.strProcrustesReferenceFile);
        refReader->Update();
        representer->SetReference(refReader->GetOutput());
    } else {
        typedef itk::VersorRigid3DTransform< float > Rigid3DTransformType;
        typedef itk::Vector<float, Dimensions> VectorType;
        typedef itk::Image<VectorType, Dimensions> ImageType;
        typedef itk::LandmarkBasedTransformInitializer<Rigid3DTransformType, ImageType, ImageType> LandmarkBasedTransformInitializerType;
        typedef itk::TransformMeshFilter< MeshType, MeshType, Rigid3DTransformType > FilterType;

        vector<MeshType::Pointer> originalMeshes;
        for (MeshList::iterator it = meshes.begin(); it != meshes.end(); it++) {
            originalMeshes.push_back(it->first);
        }

        const unsigned uMaxGPAIterations = 20;
        const unsigned uNumberOfPoints = 100;
        const float fBreakIfChangeBelow = 0.01f;
        MeshType::Pointer referenceMesh = calculateProcrustesMeanMesh<MeshType, LandmarkBasedTransformInitializerType, Rigid3DTransformType, FilterType>(originalMeshes, uMaxGPAIterations, uNumberOfPoints, fBreakIfChangeBelow);
        representer->SetReference(referenceMesh);
    }

    dataManager->SetRepresenter(representer);

    for (MeshList::const_iterator it = meshes.begin(); it != meshes.end(); it++) {
        dataManager->AddDataset(it->first, it->second.c_str());
    }

    StatisticalModelType::Pointer model;
    PCAModelBuilder::Pointer pcaModelBuilder = PCAModelBuilder::New();
    model = pcaModelBuilder->BuildNewModel(dataManager->GetData(), opt.fNoiseVariance);
    model->Save(opt.strOutputFileName.c_str());
}

po::options_description initializeProgramOptions(programOptions& poParameters) {
    po::options_description optMandatory("Mandatory options");
    optMandatory.add_options()
    ("data-list,d", po::value<string>(&poParameters.strDataListFile), "File containing a list of meshes to build shape model from")
    ("output-file,o", po::value<string>(&poParameters.strOutputFileName), "Name of the output file")
    ;
    po::options_description optAdditional("Optional options");
    optAdditional.add_options()
    ("procrustes,p", po::value<string>(&poParameters.strProcrustesMode)->default_value("GPA"), "Specify how the data is aligned: REFERENCE aligns all datasets rigidly to the reference and GPA alignes all datasets to the population mean.")
    ("reference,r", po::value<string>(&poParameters.strProcrustesReferenceFile), "Specify the reference used for model building. This is needed if --procrustes is REFERENCE")
    ("noise,n", po::value<float>(&poParameters.fNoiseVariance)->default_value(0), "Noise variance of the PPCA model")
//		("auto-noise,a", po::bool_switch(&poParameters.bAutoNoise), "Estimate noise directly from dropped components") //currently not available
    ("help,h", po::bool_switch(&poParameters.bDisplayHelp), "Display this help message")
    ;

    po::options_description optAllOptions;
    optAllOptions.add(optMandatory).add(optAdditional);
    return optAllOptions;
}