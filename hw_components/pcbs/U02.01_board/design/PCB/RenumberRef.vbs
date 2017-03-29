'
' Renumber reference designators of active sheet
'
' THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
' INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
' A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
' INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
' LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
' BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
' STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
' THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'

Dim oCurrentView : Set oCurrentView = Application.ActiveView
Dim oCurrentBlock : Set oCurrentBlock = oCurrentView.Block()
Dim sSheetName : sSheetName = oCurrentBlock.SheetNum

Call Application.AppendOutput("Renumber Refdes", "# Renumber reference designators in sheet '" & sSheetName & "' " & _
                         "[" & Year(Date) & "." & Month(Date) & "." & Day(Date) & " " & Time & "]")


' for Reference Prefix
Dim dictUIDs : Set dictUIDs = CreateObject("Scripting.Dictionary")
dictUIDS.RemoveAll

' for Location
Dim dictLocations : Set dictLocations = CreateObject("Scripting.Dictionary")
dictLocations.RemoveAll

' for X Location
Dim dictXLocations : Set dictXLocations = CreateObject("Scripting.Dictionary")
dictXLocations.RemoveAll

' for Y Location
Dim dictYLocations : Set dictYLocations = CreateObject("Scripting.Dictionary")
dictYLocations.RemoveAll

' for Old RefDes
Dim dictOldRefDes : Set dictOldRefDes = CreateObject("Scripting.Dictionary")
dictOldRefDes.RemoveAll

' for Assigning New Reference Designator
Dim dictNewRefDes : Set dictNewRefDes = CreateObject("Scripting.Dictionary")
dictNewRefDes.RemoveAll

' for Last Number used for Each Prefix
Dim dictNextRefdes : Set dictNextRefdes = CreateObject("Scripting.Dictionary")
dictNextRefdes.RemoveAll

Dim UIDArray(999)
For i=0 To UBound(UIDArray)
  UIDArray(i) = "*"
Next

Dim PrefixArray(999)
For i=0 To UBound(PrefixArray)
  PrefixArray(i) = "*"
Next
nPrefix = 0

Dim regexpRefdes : Set regexpRefdes = new Regexp
regexpRefdes.Pattern = "^([A-Z]+)(([0-9]+)|\?)"
  
'Set colComponents = Application.DesignComponents("", Application.GetProjectData.GetiCDBDesignRootBlock(Application.ActiveView.GetTopLevelDesignName()) , "-1", "STD", True)

Dim sBeginNum

Call Main

Set regexpRefdes = Nothing
Set dictNextRefdes = Nothing
Set dictNewRefDes = Nothing
Set dictOldRefDes = Nothing
Set dictYLocations = Nothing
Set dictXLocations = Nothing
Set dictLocations = Nothing
Set dictUIDs = Nothing

Sub Main()

  il = 0
  
  Dim colPins
  Set colPins = oCurrentView.Query(VDM_COMPPIN+VDM_PIN, VD_SELECTED)
  If colPins.Count > 0 Then
    'MsgBox "Unselecting " & colPins.Count & " Pins"
    For Each obj In colPins
      obj.Selected = False
    Next
  End If
  
  Dim colComponents
  Set colComponents = oCurrentView.Query(VDM_COMP, VD_SELECTED)
  If colComponents.Count < 1 Then
    Set colComponents = oCurrentView.Query(VDM_COMP, VD_ALL)
  End If
  
  For Each oComponent In colComponents
    sRefDes = ""
    sOldRefDes = ""
    enumObjectType = oComponent.Type
    If (enumObjectType = VDTS_COMPONENT) Then
      If Not oComponent.SymbolBlock.IsBorderSymbol Then
        sRefDes = getCompAttributeValue("Ref Designator", oComponent)
        If Not IsNull(sRefDes) Then
          sOldRefDes = sRefDes
          If sRefDes = "" Then
            sRefDes = getCompAttributeValue("Ref Designator", oComponent.SymbolBlock)
          End If
        Else
          sRefDes = getCompAttributeValue("Ref Designator", oComponent.SymbolBlock)
          sOldRefDes = sRefDes
        End If
      End If
    End If
    
    If Not sRefdes = "" Then
      sLocation  = oComponent.GetLocation.X/100.0 & "," & oComponent.GetLocation.Y/100.0
      sXLocation = oComponent.GetBboxPoint(VDUPPERRIGHT).X
      sYLocation = oComponent.GetBboxPoint(VDUPPERRIGHT).Y
      Call StoreComponent(sOldRefDes, sRefdes, GetContext(oComponent.UID), sLocation, sXLocation, sYLocation)
      UIDArray(il) = GetContext(oComponent.UID)
      il = il + 1
    End If
  Next
  
  If il < 1 Then
    MsgBox("No package component found in selected objects.")
    Exit Sub
  End If
  
  ' Sort UIDArray in X axis then Y axis order
  
  For i = 0 To UBound(UIDArray) - 1
    If UIDArray(i) = "*" Then
      Exit For
    End If
    For j = i+1 To UBound(UIDArray)
      If UIDArray(j) = "*" Then
        Exit For
      End If
      sXLocation_i = dictXLocations(UIDArray(i))
      sYLocation_i = dictYLocations(UIDArray(i))
      sXLocation_j = dictXLocations(UIDArray(j))
      sYLocation_j = dictYLocations(UIDArray(j))
      If sYLocation_i < sYLocation_j Then
        sUID_temp   = UIDArray(i)
        UIDArray(i) = UIDArray(j)
        UIDArray(j) = sUID_temp
      ElseIf sYLocation_i = sYLocation_j Then
        If sXLocation_i > sXLocation_j Then
          sUID_temp   = UIDArray(i)
          UIDArray(i) = UIDArray(j)
          UIDArray(j) = sUID_temp
        End If
      End If 
    Next
  Next
  
  ' Show sorted UIDs
  
  If oCurrentView.Query(VDM_COMP, VD_ALL).Count = colComponents.Count Then
    sInputBox = "Renumber *ALL* components (" & il & " ea) in the current sheet." & vbCr & vbCr
  Else
    sInputBox = "Renumber selected components (" & il & " ea) in the current sheet." & vbCr & vbCr
  End If

  sBeginNum = InputBox( sInputBox & "Enter starting number (example: 0501)", "Renumber reference designator in current sheet.")
  If Len(sBeginNum) < 1 Then
    Exit Sub
  End If
  
  Dim iBeginNum
  iBeginNum = cint(sBeginNum)
  If iBeginNum < 1 Then
    MsgBox("Input error. Number should be used for input")
    Exit Sub
  End If
  
  ' Add reference designator mapping information to dictNewRefDes
  
  For i=0 To UBound(UIDArray)
    sUID = GetContext(UIDArray(i))
    If UIDArray(i) = "*" Then
      Exit For
    End If
    
    sPrefix = dictUIDs(sUID)
    If Not dictNextRefdes.Exists(sPrefix) Then
      dictNextRefdes.Add sPrefix, iBeginNum

      PrefixArray(nPrefix) = sPrefix
      nPrefix = nPrefix + 1
    End If
  
    index = dictNextRefdes(sPrefix)
    dictNextRefdes(sPrefix) = CInt(index) + 1
    
    sOldRefDes = dictOldRefDes(sUID)
    sNewRefDes = CStr(sPrefix) & PadDigits(CStr(index))
    
    regexpRefdes.Pattern = "^([A-Z]+)(([0-9]+)|\?)$"
    Set Matches = regexpRefdes.Execute(sOldRefDes)
    
    If Not (sOldRefDes = "" or Matches.Count > 0) Then
      sNewRefDes = sOldRefDes
    Else
      dictNextRefdes(sPrefix) = CInt(index) + 1
    End If
    
    sLog = sLog & " / " & Matches.Count

    If Not dictNewRefDes.Exists(sUID) Then
      dictNewRefDes.Add sUID, sNewRefdes
    End If
  Next
  
  Set Matches = Nothing

  ' Get output file prepared

  Dim oProjectData
  Set oProjectData = Application.GetProjectData()
    
  Dim filesys, logfile
  Set filesys = CreateObject("Scripting.FileSystemObject")
  If filesys.FileExists(oProjectData.GetProjectPath + "\RenumberRefLog.txt") Then
    filesys.DeleteFile oProjectData.GetProjectPath + "\RenumberRefLog.txt"
  End If
  Set logfile = filesys.CreateTextFile(oProjectData.GetProjectPath + "\RenumberRefLog.txt", True)

  logfile.writeline "========================================================"
  logfile.writeline "DxDesigner Renumber Reference Log"
  logfile.writeline "========================================================"
  logfile.writeline "File: " & oProjectData.GetProjectPath & "\RenumberRefLog.txt"
  logfile.writeline "--------------------------------------------------------"
  logfile.writeline ""
  
  ' Assign new reference designators
  
  Application.busycursor = True
  oCurrentView.Refresh

  bSucceed = True
  
  For Each oComponent In colComponents

    sUID = GetContext(oComponent.UID)
    
    sPrefix = dictUIDs(sUID)
    
    If Not sPrefix = "" Then
      sOldRefDes = dictOldRefDes(sUID)
      sNewRefDes = dictNewRefDes(sUID)

      If sOldRefDes = sNewRefDes Then
        sLog = sUID & " : " & sOldRefDes & " (" & dictLocations(sUID) & ") : <Unchanged>"
      Else
        sLog = sUID & " : " & sOldRefDes & " -> " & sNewRefDes & " (" & dictLocations(sUID) & ") "
        Set oRefAttr = oComponent.FindAttribute("Ref Designator")
        If Not oRefAttr Is Nothing Then
          oRefAttr.Value = sNewRefDes
          If Not oRefAttr.Value = oRefAttr.EitherValue Then
            'oRefAttr.Value = getCompAttributeValue("Ref Designator", oComponent.SymbolBlock)
            'oRefAttr.InstanceValue = sNewRefDes
            oRefAttr.Delete
            oComponent.AddBatchAttributes("0 1 Ref Designator=" & sNewRefDes)
            Set oRefAttr = oComponent.FindAttribute("Ref Designator")
            oRefAttr.Selected = False
          End If
        Else
          oComponent.AddBatchAttributes("0 1 Ref Designator=" & sNewRefDes)
          Set oRefAttr = oComponent.FindAttribute("Ref Designator")
          oRefAttr.Selected = False
        End If

        If Not oRefAttr Is Nothing Then
          oRefAttr.Visible = VDVALUEVISIBLE
          If oRefAttr.EitherValue = sNewRefDes Then
            sLog = sLog & ": Succeed"
          Else
            sLog = sLog & ": Fail"
            bSucceed = False
          End If
        Else
          sLog = sLog & ": Fail"
          bSucceed = False
        End If
        
      End If
      
      Call Application.AppendOutput("Renumber Refdes", sLog)
      logfile.WriteLine(sLog)
    End If
    
  Next
  
  Set oRefAttr = Nothing
  Set colComponents = Nothing

  Call Application.AppendOutput("Renumber Refdes", "")
  Call Application.AppendOutput("Renumber Refdes", "# Numbers used by each reference prefix")
  logfile.writeline ""
  logfile.writeline "========================================================"
  logfile.writeline "Numbers used for each reference prefix"
  logfile.writeline "========================================================"

  For i = 0 To UBound(PrefixArray) - 1
    If PrefixArray(i) = "*" Then
      Exit For
    End If
    For j = i+1 To UBound(PrefixArray)
      If PrefixArray(j) = "*" Then
        Exit For
      End If
      If PrefixArray(i) > PrefixArray(j) Then
        sTmpPrefix = PrefixArray(i)
        PrefixArray(i) = PrefixArray(j)
        PrefixArray(j) = sTmpPrefix
      End If
    Next
  Next
  
  For i=0 To UBound(PrefixArray)
    sPrefix = PrefixArray(i)
    If sPrefix = "*" Then
      Exit For
    End If
    Call Application.AppendOutput("Renumber Refdes", sPrefix & " : " & PadDigits(iBeginNum) & " ~ " & PadDigits(dictNextRefdes(sPrefix) - 1))
    logfile.WriteLine(sPrefix & " : " & PadDigits(iBeginNum) & " ~ " & PadDigits(dictNextRefdes(sPrefix) - 1))
  Next
  Application.busycursor = False

  Set logfile = Nothing
  Set filesys = Nothing
  
  oCurrentView.Refresh
  If bSucceed = True Then
    MsgBox "Completed renumbering Reference Designators." & vbCr & vbCr & _
           "Please check RenumberRefLog.txt in Project Path", vbInformation, "Succeed!"
  Else
    MsgBox "Error renumbering Reference Designators." & vbCr & vbCr & _
           "Please Check RenumberRefLog.txt in Project Path." & vbCr & _
           "* Try again after running Tools > Package.", vbCritical, "Error!"
  End If

  ' Prepare WSHShell for invoking NOTEPAD.EXE
  Dim WSHSHell : Set WSHSHell=CreateObject("WScript.Shell")

  WSHSHell.Run  "%SystemRoot%\notepad.exe " & oProjectData.GetProjectPath + "\RenumberRefLog.txt"

  Set WSHSHell = Nothing

End Sub

Function PadDigits(sNum)
  Dim sRetVal : sRetVal = sNum
  Dim iNumDigits : iNumDigits = Len(sBeginNum)
  Dim iPadLen
  If iNumDigits > Len(sNum) Then
    iPadLen = iNumDigits - Len(sNum)
    sRetVal = String(iPadLen, "0") & sNum
  Else
    'sRetVal = Right(sNum, iNumDigits)
    sRetVal = CStr(sNum)
  End If
  PadDigits = sRetVal
End Function

Function GetContext(uid)
  Dim iLastSlash : iLastSlash = InStrRev(uid, "\")
  If iLastSlash > 0 Then
    GetContext = Left(uid, iLastSlash - 1)
  Else
    'GetContext = ""
    GetContext = uid
  End If
End Function

Sub StoreComponent(sOldRefDes, sRefDes, sUID, sLocation, sXLocation, sYLocation)

  Set Matches = regexpRefdes.Execute(sRefdes)
  If Not Matches.Count = 0 Then
    sPrefix = regexpRefdes.Replace(Matches(0), "$1")
  Else
    sPrefix = "U"
  End If

  If dictUIDs.Exists(sUID) Then
    dictUIDs(sUID) = sPrefix
    dictLocations(sUID) = sLocation
    dictXLocations(sUID) = sXLocation
    dictYLocations(sUID) = sYLocation
    dictOldRefDes(sUID) = sOldRefDes
  Else
    dictUIDs.Add sUID, sPrefix
    dictLocations.Add sUID, sLocation
    dictXLocations.Add sUID, sXLocation
    dictYLocations.Add sUID, sYLocation
    dictOldRefDes.Add sUID, sOldRefDes
  End If
  
  Set Matches = Nothing
  
End Sub

Function getCompAttributeValue(attrName, oComponent)
  getCompAttributeValue=Null   
  Dim attrX
  If (Not oComponent Is Nothing ) Then
    Set attrX = oComponent.FindAttribute( attrName )
    If (Not attrX Is  Nothing ) Then          
      If Len(attrX.InstanceValue) > 0 Then
        getCompAttributeValue=attrX.InstanceValue 
      Else
        getCompAttributeValue=attrX.Value
      End If  
    End If 
  End if
  Set attrX = Nothing
End Function

