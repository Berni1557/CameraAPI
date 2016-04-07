#include "QtCamListener.h"

int QtCamListener::update(cv::Mat & image)
{

	qimage = MatToQImage(image);
	// test to show QImage in QtApllication
	/*
	int argc = 1;
	char* argv[] = { "test1", "test2" };
	QApplication a(argc, argv);
	QString imagePath = QString("lena.jpg");
	QGraphicsPixmapItem item(QPixmap::fromImage(qimage));
	QGraphicsScene *scene = new QGraphicsScene();
	scene->addItem(&item);
	QGraphicsView view(scene);
	view.show();
	a.exec();
	*/

	this->onframe(qimage);
	
	return 0;
}

QImage QtCamListener::MatToQImage(const cv::Mat& mat)
{
	// 8-bits unsigned, NO. OF CHANNELS=1
	if (mat.type() == CV_8UC1)
	{
		// Set the color table (used to translate colour indexes to qRgb values)
		QVector<QRgb> colorTable;
		for (int i = 0; i<256; i++)
			colorTable.push_back(qRgb(i, i, i));
		// Copy input Mat
		const uchar *qImageBuffer = (const uchar*)mat.data;
		// Create QImage with same dimensions as input Mat
		QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
		img.setColorTable(colorTable);
		return img;
	}
	// 8-bits unsigned, NO. OF CHANNELS=1
	if (mat.type() == CV_8UC4)
	{
		QImage img(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB32);
		return img;
	}
	// 8-bits unsigned, NO. OF CHANNELS=3
	if (mat.type() == CV_8UC3)
	{
		// Copy input Mat
		const uchar *qImageBuffer = (const uchar*)mat.data;
		// Create QImage with same dimensions as input Mat
		QImage img(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
		return img.rgbSwapped();
	}
	else
	{
		qDebug() << "ERROR: Mat could not be converted to QImage.";
		return QImage();
	}
}