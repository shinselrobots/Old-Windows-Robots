
#ifndef __OSBMP_H__
#define __OSBMP_H__


class COffscreenDC : public CDC
{
public:
	// @cmember Constructs an offscreen DC for an existing screen DC
	COffscreenDC( CDC *pDC, CPalette *pPalette );
	
	virtual ~COffscreenDC();

protected:
	
	void Initialize();

	CDC *m_pDC;
	CPalette *m_pPalette, *m_pOldPalette;
	CBitmap m_osbmp, *m_oldbmp;
	CRect m_rect;

	BOOL m_bOwnDC;
};


#endif // __OSBMP_H__
