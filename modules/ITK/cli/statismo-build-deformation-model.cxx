#include <itkDirectory.h>
#include <itkImageFileReader.h>

#include "itkDataManager.h"
#include "itkPCAModelBuilder.h"
#include "itkStandardImageRepresenter.h"
#include "itkStatisticalModel.h"

#include <boost/program_options.hpp>

#include "statismo-build-models-utils.h"

namespace po = boost::program_options;


struct programOptions {
    bool bDisplayHelp;
    string strDataListFile;
    string strOutputFileName;
    float fNoiseVariance;
    bool bAutoNoise;
};

po::options_description initializeProgramOptions(programOptions& poParameters);
bool isOptionsConflictPresent(programOptions& opt);
void buildAndSaveDeformationModel(programOptions opt);



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
        buildAndSaveDeformationModel(poParameters);
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
    if (opt.strDataListFile == "" || opt.strOutputFileName == "" ) {
        return true;
    }

    if (opt.strDataListFile == opt.strOutputFileName) {
        return false;
    }

    return false;
}

void buildAndSaveDeformationModel(programOptions opt) {
    const unsigned Dimensions = 3;
    typedef itk::Image< itk::Vector<float, Dimensions>, Dimensions > ImageType;
    typedef itk::StandardImageRepresenter<itk::Vector<float, 3>, 3> RepresenterType;
    typedef itk::PCAModelBuilder<ImageType> ModelBuilderType;
    typedef itk::StatisticalModel<ImageType> StatisticalModelType;
    typedef std::vector<std::string> StringVectorType;
    typedef itk::DataManager<ImageType> DataManagerType;
    typedef itk::ImageFileReader<ImageType> ImageReaderType;
    typedef list<pair<ImageType::Pointer, string> > ImageList;

    RepresenterType::Pointer representer = RepresenterType::New();
    DataManagerType::Pointer dataManager = DataManagerType::New();

    ImageList images;
    StringList filenames = getFileList(opt.strDataListFile);
    for (StringList::const_iterator it = filenames.begin(); it != filenames.end(); it++) {
        ImageReaderType::Pointer reader = ImageReaderType::New();
        reader->SetFileName(it->c_str());
        reader->Update();
        images.push_back(make_pair(reader->GetOutput(), *it));
    }

    if (images.size() == 0) {
        itk::ExceptionObject e(__FILE__, __LINE__, "No Data was loaded and thus the model can't be built.", ITK_LOCATION);
        throw e;
    }
    representer->SetReference(images.begin()->first.GetPointer());
    dataManager->SetRepresenter(representer);

    for (ImageList::const_iterator it = images.begin(); it != images.end(); it++) {
        dataManager->AddDataset(it->first, it->second.c_str());
    }

    StatisticalModelType::Pointer model;
    ModelBuilderType::Pointer pcaModelBuilder = ModelBuilderType::New();
    model = pcaModelBuilder->BuildNewModel(dataManager->GetData(), opt.fNoiseVariance);
    model->Save(opt.strOutputFileName.c_str());
}

po::options_description initializeProgramOptions(programOptions& poParameters) {
    po::options_description optMandatory("Mandatory options");
    optMandatory.add_options()
    ("data-list,d", po::value<string>(&poParameters.strDataListFile), "File containing a list of meshes to build the deformation model from")
    ("output-file,o", po::value<string>(&poParameters.strOutputFileName), "Name of the output file")
    ;
    po::options_description optAdditional("Optional options");
    optAdditional.add_options()
    ("noise,n", po::value<float>(&poParameters.fNoiseVariance)->default_value(0), "Noise variance of the PPCA model")
//		("auto-noise,a", po::bool_switch(&poParameters.bAutoNoise), "Estimate noise directly from dropped components") //currently not available
    ("help,h", po::bool_switch(&poParameters.bDisplayHelp), "Display this help message")
    ;

    po::options_description optAllOptions;
    optAllOptions.add(optMandatory).add(optAdditional);
    return optAllOptions;
}